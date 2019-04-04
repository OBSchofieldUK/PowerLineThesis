#include "Matcher.hpp"

namespace Matcher{

    std::ostream& operator<<(std::ostream& os, const candidate& dt){
        os << "Canditate[" << dt.index << "] Angle(" << dt.angle*180/M_PI << ") Offset(" << dt.offset << ") Error(" << dt.error << ")";
        return os;
    }
    candidate::candidate():angle(0),offset(0),index(0),error(MATCHER_NO_MATCH_COST),valid(false){}
    candidate::candidate(double angle,double offset,double index):angle(angle),index(index),valid(true){
        this->offset = abs(offset);
        this->errorCal();
    }
    void candidate::errorCal(){
        this->error = ((this->offset/5))*(1+int(this->offset/100))+ this->angle*180/M_PI;
    }
    
    std::pair<std::vector<candidate>,double> match_loop(std::vector<std::vector<candidate>> &candList,int i,std::vector<bool> not_available,double solution_cost,std::vector<candidate> Solution, double &Max_solution_cost){
        if(i >= candList.size()){
            if(solution_cost < Max_solution_cost) Max_solution_cost = solution_cost;
            return std::pair<std::vector<candidate>,double>(Solution,solution_cost);
        }

        std::pair<std::vector<candidate>,double> bedst_solution;
        bedst_solution.second = 9999999;

        for(int k = 0; k < candList[i].size(); k++){
            if(solution_cost+candList[i][k].error < Max_solution_cost){
                if(!not_available[candList[i][k].index]){

                    if(candList[i][k].valid) not_available[candList[i][k].index] = true;

                    Solution[i] = candList[i][k];
                    std::pair<std::vector<candidate>,double> tmp_solution = match_loop(   candList,
                                                                                i+1,
                                                                                not_available,
                                                                                solution_cost+candList[i][k].error,
                                                                                Solution,
                                                                                Max_solution_cost
                                                                                );
                    if(tmp_solution.second< bedst_solution.second) bedst_solution = tmp_solution;

                    if(candList[i][k].valid) not_available[candList[i][k].index] = false;
                }
            }
        }
        return bedst_solution;
    }
    void matchingAlgorithm(std::vector<inspec_msg::line2d> &result, const std::vector<math::mathLine2d> &slots, const std::vector<inspec_msg::line2d> &sockets){
        std::vector<bool> matched(slots.size());
        if(!sockets.empty()){
            std::vector<std::vector<candidate>> candList(sockets.size());
            std::vector<std::vector<candidate>> curLine_candList(slots.size());


            // ############# Find Error and Prioritys between slots and sockets ###########################
            const int x_point = 1920/2;
            for(uint i = 0; i < sockets.size(); i++){
                for(uint j = 0; j < slots.size(); j++ ){
                    math::mathLine2d socket = math::ros2mathLine(sockets[i]);
                    candidate c(
                        math::vecAngle(sockets[i],mathLine2ros(slots[j])),
                        socket.b - slots[j].b,
                        j
                    );
                    if(c.error < MATCHING_LINE_MAX_ERROR){
                        candList[i].push_back(c);
                        c.index = i;
                        curLine_candList[j].push_back(c);
                    }
                }
                candList[i].push_back(candidate());
                if(candList[i].size() > 1) sort(candList[i].begin(),candList[i].end());
                /*std::cout <<endl << "##########" << " Socket: " << sockets[i].id << "########" << std::endl; 
                for(auto cand: candList[i]) cout << cand <<endl;*/
            }

            // ####################### Match The Lines #########################
            std::vector<candidate> solution(candList.size());
            double max_cost = 999999;
            std::pair<std::vector<candidate>,double> result_tmp = match_loop(candList,0,std::vector<bool>(slots.size()),0,solution,max_cost);
            for(uint i = 0; i < result_tmp.first.size(); i++){
                if(result_tmp.first[i].valid){
                    int index = result_tmp.first[i].index;
                    matched[index] = true;
                    result.push_back(mathLine2ros(slots[index],sockets[i].id));
                }
            }


            /*if(DEBUG){
                cout << endl << "############# Results ###############" << endl;
                for(int i = 0; i < result_tmp.first.size(); i++){
                    cout << result_tmp.first[i] << endl;
                }
            }*/
        }

        //################### Add unmatched Lines ##########################
        //cout << "Matching " << "Tidying up" << endl;
        for(int i = 0; i < matched.size(); i++){
            if(!matched[i]){
                result.push_back(mathLine2ros(slots[i]));
            }
        }
    }

}