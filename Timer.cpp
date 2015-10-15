//
// Created by sean on 13/10/15.
//
#include "Timer.h"

#include <limits>
#include <pcl/common/time.h>

void Timer::time() {
    double now = pcl::getTime();
    ++count_;
    if(now - last_ > 1.0) {
        std:: cout << "Average framerate (" << description_<< "): " <<
                double(count_)/(now - last_) << " Hz" << std::endl;
        count_ = 0;
        last_ = now;
    }
}

Timer::Timer(const std::string &description) {
    description_ = description;
//    last_ = std::numeric_limits<double>::lowest();
    last_ = pcl::getTime();
}
