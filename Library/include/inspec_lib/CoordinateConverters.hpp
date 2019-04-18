#ifndef COORDINATECONVERTERS_HPP_
#define COORDINATECONVERTERS_HPP_

#include <rw/math.hpp>
#include <math.h>

#include <vector>

namespace convert{
    rw::math::Quaternion<double> FRU2Image3D(rw::math::Quaternion<double> vec);
    rw::math::EAA<double> FRU2Image3D(rw::math::EAA<double> vec);
    rw::math::RPY<double> FRU2Image3D(rw::math::RPY<double> vec);
    rw::math::Rotation3D<double> FRU2Image3D(rw::math::Rotation3D<double> vec);
    rw::math::Vector3D<double> FRU2Image3D(rw::math::Vector3D<double> vec);
    rw::math::Vector3D<double> Image3D2FRU(rw::math::Vector3D<double> vec);
}

#endif
