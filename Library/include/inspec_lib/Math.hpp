#ifndef INSPEC_MATH_HPP_
#define INSPEC_MATH_HPP_

#include <inspec_msg/line2d.h>
#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <inspec_lib/CommonTypes.hpp>
#include <inspec_lib/RosConverters.hpp>
#include <eigen3/Eigen/Dense>


namespace math{
    // ############### Constructers #################
    mathLine2d constructMathLine(cv::Point2f p, double angle,cv::Mat *img = NULL);

    // ############### CONVERTERS ###################
    
    double rad2deg(const double &angle);
    double deg2rad(const double &angle);

    // ################ ALGORITHMS ##################
    mathLine2d leastSquareRegression(std::vector<cv::Point> aLine ,const cv::Size &imgSize = cv::Size(1920,1080));
    double vecAngle(const inspec_msg::line2d &l1, const inspec_msg::line2d &l2);
    double lineError(const inspec_msg::line2d &l1, const inspec_msg::line2d &l2);
    double lineError(Vector4d l1, Vector4d l2);
    double lineError(mathLine2d l1, mathLine2d l2);
    double lineError(double angle, double offset);
    double distance(const cv::Point &p1, const cv::Point p2 = cv::Point(0,0));
    double distance(const cv::Point &p, const mathLine2d &l);
    double distance(const mathLine2d l, const cv::Point p);
    
    // ################ DEBUG #######################
    void drawMathLine(cv::Mat &dst, mathLine2d line, cv::Scalar color = cv::Scalar(255,255,255),std::string text = "",cv::Scalar textColor = cv::Scalar(255,255,255));
    std::ostream& operator<<(std::ostream& os, const mathLine2d& dt);
    std::ostream& operator<<(std::ostream& os, const inspec_msg::line2d& dt);
}


#endif