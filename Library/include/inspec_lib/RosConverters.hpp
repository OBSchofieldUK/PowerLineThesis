#ifndef ROSCONVERTERS_HPP_
#define ROSCONVERTERS_HPP_

#include <inspec_lib/CommonTypes.hpp>
#include <inspec_msg/line2d.h>
#include <inspec_msg/line2d_array.h>
#include <inspec_msg/line3d_array.h>

#include <rw/math.hpp>
#include <math.h>

#include <vector>

namespace convert{
    math::mathLine2d ros2mathLine(inspec_msg::line2d line);
    inspec_msg::line2d mathLine2ros(math::mathLine2d mline, uint id = 0);

    rw::math::Vector3D<double> ros2Vector3D(boost::array<double, 3ul> vec);
    boost::array<double, 3ul> Vector3D2ros(rw::math::Vector3D<double> vev);
    rw::math::Quaternion<double> ros2Quaternion(boost::array<double, 4ul> vec);
    boost::array<double, 4ul> Quaternion2ros(rw::math::Quaternion<double> vec);
    boost::array<double, 4ul> Quaternion2ros(rw::math::RPY<double> vec);
    boost::array<double, 4ul> Quaternion2ros(rw::math::Rotation3D<double> vec);
    boost::array<double, 4ul> Quaternion2ros(rw::math::EAA<double> vec);

    rw::math::Quaternion<double> FRU2Image3D(rw::math::Quaternion<double> vec);
    rw::math::EAA<double> FRU2Image3D(rw::math::EAA<double> vec);
    rw::math::RPY<double> FRU2Image3D(rw::math::RPY<double> vec);
    rw::math::Rotation3D<double> FRU2Image3D(rw::math::Rotation3D<double> vec);
    rw::math::Vector3D<double> FRU2Image3D(rw::math::Vector3D<double> vec);
    rw::math::Vector3D<double> Image3D2FRU(rw::math::Vector3D<double> vec);

    inspec_msg::line2d line2ros(const math::Vector4d &line);
    inspec_msg::line3d line2ros(const math::Vector7d &line);
    math::Vector4d ros2line(const inspec_msg::line2d line);

    inspec_msg::line2d_array line2d_array_construct(std::vector<inspec_msg::line2d> lines = std::vector<inspec_msg::line2d>(), uint seq = 0, ros::Time time = ros::Time(0));

}

#endif