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

    std::vector<boost::shared_ptr<pcl::visualization::PCLVisualizer>> cloud_viewers_;
    std::vector<boost::shared_ptr<pcl::visualization::ImageViewer>> image_viewers_;
    std::vector<boost::shared_ptr<ViewerNode<PointType>>> nodes_;

//    size_t current_cloud_;
    std::vector<bool> cloud_init_;
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

//        if(event.getKeyCode() == 'n' && !event.keyDown())
//            current_cloud_ = (current_cloud_+1) % nodes_.size();
//
//        if(event.getKeyCode() == 'N' && !event.keyDown())
//            current_cloud_ = (current_cloud_ + nodes_.size() - 1) % nodes_.size();
    }

    void mouseCallback(const pcl::visualization::MouseEvent& mouse_event, void*) {
        if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton) {
            cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
        }
    }

    bool checkCloseCondition() {
        for(auto cv: cloud_viewers_)
            if(cv->wasStopped())
                return true;

        // NB: No elements in the vector if no nodes provide image callback
        for(auto iv: image_viewers_)
            if(iv->wasStopped())
                return true;

        return false;
    }

    void updateCloudViewer(size_t node_index) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> current_viewer(cloud_viewers_.at(node_index));
        current_viewer->spinOnce ();

        CloudConstPtr cloud;

        nodes_.at(node_index)->getCloud(cloud);

        if(cloud) {
            if (!cloud_init_.at(node_index)) {
                current_viewer->setPosition (1280 * (int)node_index, 480 + 25);
                current_viewer->setSize (cloud->width, cloud->height);
                int *size = current_viewer->getRenderWindow()->GetScreenSize();
                cout << "Size " << size[0] << ", " << size[1] << endl;
                cloud_init_.at(node_index) = !cloud_init_.at(node_index);
            }

            if (!current_viewer->updatePointCloud (cloud, "OpenNICloud")) {
                current_viewer->addPointCloud (cloud, "OpenNICloud");
                current_viewer->resetCameraViewpoint ("OpenNICloud");
                current_viewer->setCameraPosition (
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
            : cloud_viewers_()
            , image_viewers_ () {
        int id = 0;
        for(auto& g: grabbers) {
            boost::shared_ptr<ViewerNode<PointType>> node(new ViewerNode<PointType>(g, id));
            nodes_.push_back(node);
            image_init_.push_back(false);
            cloud_init_.push_back(false);
            ++id;
        }
    }

    void run() {
        for(size_t i=0; i<nodes_.size(); ++i) {
            std::stringstream ss;
            ss << "Camera " << i;
            boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer
                    (new pcl::visualization::PCLVisualizer(ss.str() + " Cloud"));
            cloud_viewer->registerMouseCallback(&MultiOpenNI2Viewer::mouseCallback, *this);
            cloud_viewer->registerKeyboardCallback(&MultiOpenNI2Viewer::keyboardCallback, *this);
            // TODO: Check this number for Asus Xtion
            cloud_viewer->setCameraFieldOfView(1.02259994f);
            cloud_viewers_.push_back(cloud_viewer);

            nodes_.at(i)->setup();

            if(nodes_.at(i)->providesImageCallback()) {
                boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer
                        (new pcl::visualization::ImageViewer (ss.str() + " Image"));
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
            for(size_t i=0; i<nodes_.size(); ++i) {
                updateCloudViewer(i);
                updateImageViewer(i);
            }
        }

        for(auto n: nodes_)
            n->stop();
    }
};

#endif //PCL_MULTI_CAMERA_REGISTRATION_MULTIOPENNI2VIEWER_H
