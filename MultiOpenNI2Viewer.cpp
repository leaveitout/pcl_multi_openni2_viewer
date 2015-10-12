//
// Created by sean on 07/10/15.
//

#include "MultiOpenNI2Viewer.h"
#include "ViewerNode.h"

template <typename PointType>
MultiOpenNI2Viewer<PointType>::MultiOpenNI2Viewer(std::vector<pcl::io::OpenNI2Grabber::Ptr>& grabbers)
}

template <typename PointType>
void MultiOpenNI2Viewer<PointType>::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *pVoid) {
}

template <typename PointType>
void MultiOpenNI2Viewer<PointType>::mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *pVoid) {
}

template <typename PointType>
void MultiOpenNI2Viewer<PointType>::run() {

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

template <typename PointType>
bool MultiOpenNI2Viewer<PointType>::checkCloseCondition() {
}

template <typename PointType>
void MultiOpenNI2Viewer<PointType>::updateCloudViewer() {

}

template <typename PointType>
void MultiOpenNI2Viewer<PointType>::updateImageViewer(size_t node_index) {

}
