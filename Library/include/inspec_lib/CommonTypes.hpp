#ifndef INSPEC_COMMONTYPES_HPP_
#define INSPEC_COMMONTYPES_HPP_

#include <eigen3/Eigen/Dense>

namespace math{
    struct mathLine2d{
        double a;
        double b;
    };
    typedef Eigen::Matrix<double,7,1> Vector7d;
    typedef Eigen::Matrix<double,4,1> Vector4d;
    typedef Eigen::Matrix<double,7,7> Matrix7;
    typedef Eigen::Matrix<double,4,4> Matrix4;
    typedef Eigen::Matrix<double,4,7> Matrix4x7;
    typedef Eigen::Matrix<double,7,4> Matrix7x4;
}


#endif