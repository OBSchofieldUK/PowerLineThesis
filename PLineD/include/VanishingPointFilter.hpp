#ifndef VANNISHING_POINT_FILTER_HPP_
#define VANNISHING_POINT_FILTER_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <inspec_lib/Math.hpp>
#include <iostream>
#include <inspec_lib/settings/SettingStructs.hpp>

namespace VP{

    extern settings::Vannishing_Point_Filter setting;
    struct VPoint{
        uint ID1;
        uint ID2;
        cv::Point p;
    };
    struct Cand{
        VPoint p;
        double error;
    };
    static struct candComparetor {
        bool operator() (Cand i,Cand j) { 
            return (i.error<j.error);
        }
    } candComp;

    void filterLines(std::vector<math::mathLine2d> src,std::vector<math::mathLine2d> &dst,double min_point2line_error, double max_point_cluster_error);
    VPoint calculateVanishingPoint(const math::mathLine2d &l1, const math::mathLine2d &l2);
    void calculateVanishingPoint(const std::vector<math::mathLine2d> &src,std::vector<VPoint> &dst);
    void buildClusterData(const std::vector<VPoint> &src, std::vector<std::vector<Cand>> &dst);
    bool findClusterCenter(std::vector<std::vector<Cand> > src, Cand &dst,uint error_check_index, double max_error = 300, double min_error = 50);
}
#endif
