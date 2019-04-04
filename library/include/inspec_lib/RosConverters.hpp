#ifndef ROSCONVERTERS_HPP_
#define ROSCONVERTERS_HPP_

#include "Math.hpp"
#include <inspec_msg/line2d.h>

namespace converter{
    math::mathLine2d ros2mathLine(inspec_msg::line2d line);
    inspec_msg::line2d mathLine2ros(math::mathLine2d mline, uint id = 0);
}

#endif