#include "inspec_lib/converters/OtherConverters.hpp"

namespace convert{
    rw::math::Transform3D<double> ToTransform(double XYZ[3], double RPY[3]){
        return rw::math::Transform3D<double>(  rw::math::Vector3D<double>(
                                            XYZ[0],
                                            XYZ[1],
                                            XYZ[2]),
                                        rw::math::RPY<double>(
                                            math::deg2rad(RPY[2]),
                                            math::deg2rad(RPY[1]),
                                            math::deg2rad(RPY[0])
                                        ).toRotation3D()
                                    );
    }
    rw::math::Transform3D<double> ToTransform(std::vector<double> XYZ, std::vector<double> RPY){
        return rw::math::Transform3D<double>(  rw::math::Vector3D<double>(
                                            XYZ[0],
                                            XYZ[1],
                                            XYZ[2]),
                                        rw::math::RPY<double>(
                                            math::deg2rad(RPY[2]),
                                            math::deg2rad(RPY[1]),
                                            math::deg2rad(RPY[0])
                                        ).toRotation3D() //ROLL and Yaw are switched in RobWork
                                    );
    }
}