#include "inspec_lib/converters/OtherConverters.hpp"

namespace convert{
    rw::math::Transform3D<double> ToTransform(double XYZ[3], double RPY[3]){
        return rw::math::Transform3D<double>(  rw::math::Vector3D<double>(
                                            XYZ[0],
                                            XYZ[1],
                                            XYZ[2]),
                                        rw::math::RPY<double>(
                                            RPY[0],
                                            RPY[1],
                                            RPY[2]
                                        ).toRotation3D()
                                    );
    }
    rw::math::Transform3D<double> ToTransform(std::vector<double> XYZ, std::vector<double> RPY){
        return rw::math::Transform3D<double>(  rw::math::Vector3D<double>(
                                            XYZ[0],
                                            XYZ[1],
                                            XYZ[2]),
                                        rw::math::RPY<double>(
                                            RPY[0],
                                            RPY[1],
                                            RPY[2]
                                        ).toRotation3D()
                                    );
    }
}