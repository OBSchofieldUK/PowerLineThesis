#ifndef lineSeg
#define lineSeg std::vector< std::vector<cv::Point> >
#endif

#ifndef PROXIMITY_FILTERING_HPP
#define PROXIMITY_FILTERING_HPP

#include <opencv2/opencv.hpp>
#include <inspec_lib/Math.hpp>
#include <PLineD.hpp>

#include <string>
#include <vector>


//typedef unsigned int uint;
namespace Prox{
    extern bool DEBUG_PROXIMITY_FILTER;
    
    const uint RIGHT = 1, LEFT = 2, UP = 3, DOWN = 4;
    struct boxLine{
        cv::Point2f left;
        cv::Point2f right;
    };
    // ############# Helper Functions ###################
    void correctBBox(cv::RotatedRect &BBox);
    cv::Point findBoxEnd(cv::RotatedRect BBox,uint side);

    void filter(lineSeg& lines,cv::Mat DEBUG_img = cv::Mat(), double EXTENSION_MAX_ANGLE = 7.0, double PARRALEL_MAX_ANGLE = 4.0, double ADDTION_MAX_DISTANCE = 14.0, double EXTENTSION_MAX_DISTANCE = 10.0, double CENTER2LINE_DISTANCE_MAX = 50.0);
    void findBoxes(lineSeg& src, std::vector<cv::RotatedRect> &dst, cv::Mat DEBUG_img = cv::Mat());
    void findLines2Filter(std::vector<cv::RotatedRect> &src, std::vector<uint> &remove, std::vector<std::pair<uint,uint> > &gather,double EXTENSION_MAX_ANGLE = 7.0, double PARRALEL_MAX_ANGLE = 4.0, double ADDTION_MAX_DISTANCE = 14.0, double EXTENTSION_MAX_DISTANCE = 10.0, double CENTER2LINE_DISTANCE_MAX = 50.0);

    bool isExtensionLine(const boxLine &main, const boxLine &secondary, math::mathLine2d &Lmain, double &distReturn);
    std::pair<uint,uint> larger(lineSeg &Lines,uint i1, uint i2);

    void applyFilterChanges(lineSeg &lines, std::vector<uint> &remove, std::vector<std::pair<uint,uint> > &gather);
}
#endif