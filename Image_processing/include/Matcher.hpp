#ifndef MATCHER_HPP_
#define MATCHER_HPP_

#include <inspec_msg/line2d.h>
#include <iostream>
#include <vector>

#include <inspec_lib/Math.hpp>
#include <inspec_lib/converters/RosConverters.hpp>

namespace Matcher{
    extern double LINE_MAX_ERROR;
    extern double NO_MATCH_COST;
    // ########### Data TYPES #############

    class candidate{

     public:
        candidate();
        candidate(double angle,double offset,double index);
        void errorCal();
        bool operator< (const candidate &rhs) const{
            return this->error < rhs.error;
        }
        double angle;
        double offset;
        double error;
        uint index;
        bool valid;
    };

    // ######### ALGORITHMS ###########
    void matchingAlgorithm(std::vector<inspec_msg::line2d> &result, const std::vector<math::mathLine2d> &slots, const std::vector<inspec_msg::line2d> &sockets);
    std::pair<std::vector<candidate>,double> match_loop(std::vector<std::vector<candidate> > &candList,int i,std::vector<bool> not_available,double solution_cost,std::vector<candidate> Solution, double &Max_solution_cost);
    
    // ########## Other ###############
    std::ostream& operator<<(std::ostream& os, const candidate& dt);
}
#endif