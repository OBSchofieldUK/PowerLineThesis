#ifndef INSPEC_MATH_HPP_
#define INSPEC_MATH_HPP_

#include <inspec_msg/line2d.h>
#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>

namespace math{
    struct mathLine2d{
        double a;
        double b;
    };

    // ############### CONVERTERS ###################
    
    double rad2deg(const double &angle);
    double deg2rad(const double &angle);

    // ################ ALGORITHMS ##################
    mathLine2d leastSquareRegression(std::vector<cv::Point> aLine ,const cv::Size &imgSize = cv::Size(1920,1080));
    double vecAngle(const inspec_msg::line2d &l1, const inspec_msg::line2d &l2);
    
    // ################ DEBUG #######################
    void drawMathLine(cv::Mat &dst, mathLine2d line, cv::Scalar color = cv::Scalar(255,255,255),std::string text = "",cv::Scalar textColor = cv::Scalar(255,255,255));
    std::ostream& operator<<(std::ostream& os, const mathLine2d& dt);
    std::ostream& operator<<(std::ostream& os, const inspec_msg::line2d& dt);
}


#endif
