//
// Created by sean on 07/10/15.
//

#ifndef PCL_MULTI_CAMERA_REGISTRATION_MULTIOPENNI2VIEWER_H
#define PCL_MULTI_CAMERA_REGISTRATION_MULTIOPENNI2VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni2_grabber.h>

#include "ViewerNode.h"

template <typename PointType>
class MultiOpenNI2Viewer {

private:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    std::vector<boost::shared_ptr<pcl::visualization::ImageViewer>> image_viewers_;
    std::vector<boost::shared_ptr<ViewerNode<PointType>>> nodes_;

    size_t current_cloud_;
    bool cloud_init_;
    std::vector<bool> image_init_;

    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*) {
        if (event.getKeyCode ())
            cout << "the key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
        else
            cout << "the special key \'" << event.getKeySym () << "\' was";
        if (event.keyDown ())
            cout << " pressed" << endl;
        else
            cout << " released" << endl;

        if(event.getKeyCode() == 'n' && !event.keyDown())
            current_cloud_ = (current_cloud_+1) % nodes_.size();

        if(event.getKeyCode() == 'N' && !event.keyDown())
            current_cloud_ = (current_cloud_ + nodes_.size() - 1) % nodes_.size();
    }

    void mouseCallback(const pcl::visualization::MouseEvent& mouse_event, void*) {
        if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton) {
            cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
        }
    }

    bool checkCloseCondition() {
        if(cloud_viewer_->wasStopped())
            return true;

        // NB: No elements in the vector if no nodes provide image callback
        for(auto iv: image_viewers_)
            if(iv->wasStopped())
                return true;

        return false;
    }

    void updateCloudViewer() {
        cloud_viewer_->spinOnce ();

        CloudConstPtr cloud;

        nodes_.at(current_cloud_)->getCloud(cloud);

        if(cloud) {
            if (!cloud_init_) {
                cloud_viewer_->setPosition (0, 0);
//                cloud_viewer_->setSize (cloud->width, cloud->height);
                cloud_init_ = !cloud_init_;
            }

            if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud")) {
                cloud_viewer_->addPointCloud (cloud, "OpenNICloud");
                cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
                cloud_viewer_->setCameraPosition (
                        0,0,0,		// Position
                        0,0,1,		// Viewpoint
                        0,-1,0);	// Up
            }
        }
    }

    void updateImageViewer(size_t node_index) {
        boost::shared_ptr<pcl::io::openni2::Image> image;
        // See if we can get an image
        nodes_.at(node_index)->getImage(image);

        if (image) {
            if (!image_init_.at(node_index)) {
                int count_init = 0;
                for(auto ii: image_init_)
                    if(ii)
                        count_init++;
                // TODO: Assumes all the images are of the same width
                image_viewers_.at(node_index)->setPosition(image->getWidth()*count_init, 0);
                image_viewers_.at(node_index)->setSize (image->getWidth(), image->getHeight());
                image_init_.at(node_index) = !image_init_.at(node_index);
            }

            if (image->getEncoding () == pcl::io::openni2::Image::RGB)
                image_viewers_.at(node_index)->addRGBImage ( (const unsigned char*)image->getData (), image->getWidth (), image->getHeight ());
            // TODO: Give access to rgb_data_ of the node
//            else
//                image_viewers_.at(node_index)->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
            image_viewers_.at(node_index)->spinOnce ();
        }
    }

public:

    MultiOpenNI2Viewer(std::vector<pcl::io::OpenNI2Grabber::Ptr>& grabbers)
            : cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL MultiCloud Viewer"))
            , image_viewers_ ()
            , current_cloud_ (0)
            , cloud_init_ (false) {
        for(auto g: grabbers) {
            boost::shared_ptr<ViewerNode<PointType>> node(new ViewerNode<PointType>(g));
            nodes_.push_back(node);
            image_init_.push_back(false);
        }
    }

    void run() {
        cloud_viewer_->registerMouseCallback(&MultiOpenNI2Viewer::mouseCallback, *this);
        cloud_viewer_->registerKeyboardCallback(&MultiOpenNI2Viewer::keyboardCallback, *this);
        // TODO: Check this number for Asus Xtion
        cloud_viewer_->setCameraFieldOfView(1.02259994f);

        for(size_t i=0; i<nodes_.size(); ++i) {
            nodes_.at(i)->setup();
            if(nodes_.at(i)->providesImageCallback()) {
                std::stringstream ss;
                ss << "Camera " << i << " Image";
                boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer
                        (new pcl::visualization::ImageViewer (ss.str()));
                image_viewer->registerMouseCallback (&MultiOpenNI2Viewer::mouseCallback, *this);
                image_viewer->registerKeyboardCallback (&MultiOpenNI2Viewer::keyboardCallback, *this);
                image_viewers_.push_back(image_viewer);
            }
            else
                image_viewers_.push_back(nullptr);
        }

        for(auto n: nodes_)
            n->start();

        while(!checkCloseCondition()) {
            updateCloudViewer();
            for(size_t i=0; i<nodes_.size(); ++i)
                updateImageViewer(i);
        }

        for(auto n: nodes_)
            n->stop();
    }
};

#endif //PCL_MULTI_CAMERA_REGISTRATION_MULTIOPENNI2VIEWER_H
