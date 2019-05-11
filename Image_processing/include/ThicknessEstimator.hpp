#ifndef lineSeg
#define lineSeg std::vector< std::vector<cv::Point> >
#endif

#ifndef THICKNESS_ESTIMATOR_HPP_
#define THICKNESS_ESTIMATOR_HPP_

#include <iostream>
#include <vector>
#include <deque>
#include <inspec_lib/Math.hpp>
#include <inspec_lib/converters/CoordinateConverters.hpp>
#include <opencv2/opencv.hpp>

#include <math.h>


namespace ThickEst{
    typedef std::deque< std::vector<float> > vvi;
    typedef std::vector<float>  vi;
    typedef std::vector< cv::Point> vp;
    typedef std::vector< cv::Point2f> vpf;
    void findParallelPixels(const vp &src,const math::mathLine2d &line, vvi &dst, const cv::Size &imgSize = cv::Size(1920,1080));
    void findThickness(vvi &src, vpf &dst);
    float minDist(float point, const vi &options);
    void print(const vvi &src);
}
#endif