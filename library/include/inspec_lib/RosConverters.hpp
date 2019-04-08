#ifndef ROSCONVERTERS_HPP_
#define ROSCONVERTERS_HPP_

#include <inspec_lib/Math.hpp>
#include <inspec_msg/line2d.h>
#include <inspec_msg/line2d_array.h>

#include <rw/math.hpp>

#include <vector>

namespace converter{
    math::mathLine2d ros2mathLine(inspec_msg::line2d line);
    inspec_msg::line2d mathLine2ros(math::mathLine2d mline, uint id = 0);

    rw::math::Vector3D<double> ros2Vector3D(boost::array<double, 3ul> vec);
    rw::math::Quaternion<double> ros2Quaternion(boost::array<double, 4ul> vec);


    inspec_msg::line2d_array line2d_array_construct(std::vector<inspec_msg::line2d> lines = std::vector<inspec_msg::line2d>(), uint seq = 0, ros::Time time = ros::Time(0));

}

#endif