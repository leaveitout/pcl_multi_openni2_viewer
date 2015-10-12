//
// Created by sean on 8/10/15.
//

//#include "ViewerNode.h"

template <typename PointType>
ViewerNode<PointType>::ViewerNode(pcl::io::OpenNI2Grabber::Ptr grabber)

template <typename PointType>

template <typename PointType>
void ViewerNode<PointType>::imageCallback(const boost::shared_ptr<pcl::io::openni2::Image>& image) {
    boost::mutex::scoped_lock lock (image_mutex_);
    image_ = image;

    if (image->getEncoding () != pcl::io::openni2::Image::RGB) {
        if (rgb_data_size_ < image->getWidth () * image->getHeight ()) {
            if (rgb_data_)
                delete [] rgb_data_;
            rgb_data_size_ = image->getWidth () * image->getHeight ();
            rgb_data_ = new unsigned char [rgb_data_size_ * 3];
        }
        image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
    }
}

template <typename PointType>
void ViewerNode<PointType>::setup() {
}

template <typename PointType>
void ViewerNode<PointType>::start() {
    grabber_->start();
}

template <typename PointType>
void ViewerNode<PointType>::stop() {
    grabber_->stop ();

    cloud_connection_.disconnect ();
    image_connection_.disconnect ();

    if (rgb_data_)
        delete[] rgb_data_;
}

template <typename PointType>
bool ViewerNode<PointType>::providesImageCallback() {

template <typename PointType>
bool ViewerNode<PointType>::getCloud(ViewerNode::CloudConstPtr cloud) {
}

template <typename PointType>
bool ViewerNode<PointType>::getImage(boost::shared_ptr<pcl::io::openni2::Image> image) {
}
