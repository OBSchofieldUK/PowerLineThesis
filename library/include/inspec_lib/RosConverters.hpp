#ifndef ROSCONVERTERS_HPP_
#define ROSCONVERTERS_HPP_

#include <inspec_lib/Math.hpp>
#include <inspec_msg/line2d.h>

#include <rw/math.hpp>

#include <vector>

namespace converter{
    math::mathLine2d ros2mathLine(inspec_msg::line2d line);
    inspec_msg::line2d mathLine2ros(math::mathLine2d mline, uint id = 0);

    rw::math::Vector3D<double> ros2Vector3D(std::vector<double> vec);
    rw::math::Quaternion<double> ros2Quaternion(std::vector<double> vec);

}

#endif