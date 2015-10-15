//
// Created by sean on 13/10/15.
//

#ifndef PCL_MULTI_OPENNI2_VIEWER_TIMER_H
#define PCL_MULTI_OPENNI2_VIEWER_TIMER_H

#include <string>


class Timer {
private:
    double last_;
    std::string description_;
    int count_;

public:
    Timer(const std::string& description);
    void time();

};


#endif //PCL_MULTI_OPENNI2_VIEWER_TIMER_H
