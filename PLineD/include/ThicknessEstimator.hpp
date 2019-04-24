#ifndef lineSeg
#define lineSeg std::vector< std::vector<cv::Point> >
#endif

#ifndef THICKNESS_ESTIMATOR_HPP_
#define THICKNESS_ESTIMATOR_HPP_

#include <iostream>
#include <vector>
#include <deque>
#include <inspec_lib/Math.hpp>
#include <opencv2/opencv.hpp>


namespace ThickEst{
    typedef std::deque< std::vector<int> > vvi;
    typedef std::vector<int>  vi;
    typedef std::vector< cv::Point> vp;
    void findParallelPixels(const vp &src,const math::mathLine2d &line, vvi &dst, const cv::Size &imgSize = cv::Size(1920,1080));
    void findThickness(vvi &src, vp &dst);
    int minDist(int point, const vi &options);
    void print(const vvi &src);
}
#endif