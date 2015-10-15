//
// Created by sean on 08/10/15.
//

#ifndef PCL_MULTI_CAMERA_REGISTRATION_VIEWERNODE_H
#define PCL_MULTI_CAMERA_REGISTRATION_VIEWERNODE_H


#include <pcl/io/openni2_grabber.h>

#include "Timer.h"


template <typename PointType>
class ViewerNode {
private:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    pcl::io::OpenNI2Grabber::Ptr grabber_;

    boost::signals2::connection cloud_connection_;
    boost::signals2::connection image_connection_;

    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;

    CloudConstPtr cloud_;
    boost::shared_ptr<pcl::io::openni2::Image> image_;
    unsigned char *rgb_data_;
    unsigned rgb_data_size_;

    boost::shared_ptr<Timer> cloud_timer_, image_timer_;
    const int id_;

    void cloudCallback(const CloudConstPtr& cloud) {
        cloud_timer_->time();
        boost::mutex::scoped_lock lock(cloud_mutex_);
        cloud_ = cloud;
    }

    void imageCallback(const boost::shared_ptr<pcl::io::Image> &image) {
        image_timer_->time();
        boost::mutex::scoped_lock lock(image_mutex_);
        image_ = image;

        if (image->getEncoding() != pcl::io::openni2::Image::RGB) {
            if (rgb_data_size_ < image->getWidth() * image->getHeight()) {
                if (rgb_data_)
                    delete[] rgb_data_;
                rgb_data_size_ = image->getWidth() * image->getHeight();
                rgb_data_ = new unsigned char[rgb_data_size_ * 3];
            }
            image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
        }
    }

public:

    ViewerNode(pcl::io::OpenNI2Grabber::Ptr grabber, int id)
            : grabber_ (grabber)
            , id_ (id)
            , rgb_data_ ()
            , rgb_data_size_ () {
        std::stringstream ss1;
        ss1 << "Camera " << id << " ";
        cloud_timer_.reset(new Timer(ss1.str().append("Cloud Timer")));
        image_timer_.reset(new Timer(ss1.str().append("Image TImer")));
    }

    void setup() {
        boost::function<void (const CloudConstPtr&) > cloud_cb =
                boost::bind(&ViewerNode::cloudCallback, this, _1);
        cloud_connection_ = grabber_->registerCallback(cloud_cb);

        if (grabber_->providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>()) {
            boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb =
                    boost::bind (&ViewerNode::imageCallback, this, _1);
            image_connection_ = grabber_->registerCallback (image_cb);
        }
    }

    void start() {
        grabber_->start();
    }

    void stop() {
        grabber_->stop ();

        cloud_connection_.disconnect ();
        image_connection_.disconnect ();

        if (rgb_data_)
            delete[] rgb_data_;
    }

    bool providesImageCallback() {
        if (grabber_->providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>() )
            return true;
        else
            return false;
    }

    void getCloud(CloudConstPtr& cloud) {
        if(cloud_mutex_.try_lock()) {
            cloud_.swap(cloud);
            cloud_mutex_.unlock();
        }
    }

    bool getImage(boost::shared_ptr<pcl::io::openni2::Image>& image) {
        if(!grabber_->isRunning())
            cout << "Not running!" << endl;

        if(providesImageCallback())
            if(image_mutex_.try_lock()) {
                image_.swap(image);
                image_mutex_.unlock();
                return true;
            }
        return false;
    }
};

#endif //PCL_MULTI_CAMERA_REGISTRATION_VIEWERNODE_H
