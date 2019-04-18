#include <inspec_lib/CoordinateConverters.hpp>

namespace convert{
    rw::math::Vector3D<double> FRU2Image3D(rw::math::Vector3D<double> vec){
        return rw::math::Vector3D<double>(vec[1],vec[2],vec[0]);
    }
    rw::math::Vector3D<double> Image3D2FRU(rw::math::Vector3D<double> vec){
        return rw::math::Vector3D<double>(vec[2],vec[0],vec[1]);
    }
    // ###### FRU Angle Converters ######
    rw::math::Quaternion<double> FRU2Image3D(rw::math::Quaternion<double> vec){
        return rw::math::Quaternion<double>(FRU2Image3D(vec.toRotation3D()));
    }
    rw::math::EAA<double> FRU2Image3D(rw::math::EAA<double> vec){
        return rw::math::EAA<double>(FRU2Image3D(vec.axis()),vec.angle());
    }
    rw::math::RPY<double> FRU2Image3D(rw::math::RPY<double> vec){
        return rw::math::RPY<double>(FRU2Image3D(vec.toRotation3D()));
    }
    rw::math::Rotation3D<double> FRU2Image3D(rw::math::Rotation3D<double> vec){
        return FRU2Image3D(rw::math::EAA<double>(vec)).toRotation3D();
    }
}

