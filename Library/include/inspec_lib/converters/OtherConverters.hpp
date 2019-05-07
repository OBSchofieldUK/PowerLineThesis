#ifndef OTHER_CONVERTERS_HPP_
#define OTHER_CONVERTERS_HPP_

#include <rw/math.hpp>
#include <inspec_lib/Math.hpp>
#include <vector>
namespace convert{
    rw::math::Transform3D<double> ToTransform(double XYZ[3], double RPY[3]);
    rw::math::Transform3D<double> ToTransform(std::vector<double> XYZ, std::vector<double> RPY);
}

#endif