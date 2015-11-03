#include <pcl/io/openni2_grabber.h>

#include "MultiOpenNI2Viewer.h"

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

using namespace std;

int main(int argc, char** argv) {

    // TODO: Some argument parsing for grabber modes, etc. See PCL sample
    auto device_manager = NI2DeviceManager::getInstance();

    vector<NI2Grabber::Ptr> grabbers;

    size_t numDevices = device_manager->getNumOfConnectedDevices();
    for(size_t i=1; i<=numDevices; ++i) {
        stringstream ss;
        ss << '#' << i;
        NI2Grabber::Ptr grabber(new NI2Grabber(ss.str()));
        grabbers.push_back(grabber);
    }

    for(auto g: grabbers) {
        cout << g->getDevice()->getUri() << endl;
    }

    MultiOpenNI2Viewer<pcl::PointXYZRGBA> viewer(grabbers);

    viewer.run();

    return 0;
}
