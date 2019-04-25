#include "ThicknessEstimator.hpp"

namespace ThickEst{
    void findParallelPixels(const vp &src,const math::mathLine2d &line, vvi &dst, const cv::Size &imgSize ){
        dst = vvi(imgSize.width);
        int first=imgSize.width,last=0;
        for(auto p: src){
            cv::Point2f point; 
            double angle = std::atan(line.a);
            point.x = std::cos(angle)*p.x - sin(angle)*p.y;
            point.y = std::sin(angle)*p.x + cos(angle)*p.y;
            std::cout << point << p << std::endl; 
            bool notThere = true;
            
            for(uint i = 0; i < dst[point.x].size(); i++){
                if(dst[point.x][i] == point.y){
                    notThere = false;
                    break;
                }
            }
            if(notThere){
                dst[size_t(point.x)].push_back(point.y);
                if(dst[point.x].size()>1){
                    if(point.x < first) first = point.x;
                    if(point.x > last) last = point.x;
                }
            }
        }

        /*if(first > 1) dst.erase(dst.begin(),dst.begin()+first-1);
        else if(first == 1) dst.erase(dst.begin());

        if(last < dst.size()-1)dst.erase(dst.begin()+last+1,dst.end());
        else if(last == dst.size()-1) dst.erase(dst.end());*/

        dst.push_front({int(first-1)});
        //std::cout << "First: " << first << std::endl;

    }
    int minDist(int point, const vi &options){
        int min = 5000;
        for(uint i = 0; i < options.size(); i++){
            int dist = std::abs(options[i]-point);
            if(dist < min){
                min = dist;
                if(dist == 0) break;
            }
        }
        return min;
    }
    void findThickness(vvi &src, vp &dst){
        for(uint i = 1; i < src.size(); i++){
            int y = -1;
            if(src[i].size()== 2){
                y = std::abs(src[i][0]-src[i][1]);
                if(y < 2){ 
                    y = -1;
                    src[i] = {};
                }
            }else if(src[i].size() > 2){
                uint index1, index2;
                uint error_min = -1;
                uint min_dist = -1;
                for(uint e = 0; e < src[i].size(); e++){
                    for(uint k = e+1; k < src[i].size(); k++){
                        int dist = std::abs(src[i][e]-src[i][k]);
                        if(dist > 1){
                            int error = 0;
                            if(i>2){
                                error+= minDist(src[i][e],src[i-1]);
                                error+= minDist(src[i][k],src[i-1]);
                            } 
                            if(i < src.size()-1){
                                error+= minDist(src[i][e],src[i+1]);
                                error+= minDist(src[i][k],src[i+1]);
                            }
                            if(error < error_min || (error == error_min && dist < min_dist)){
                                index1 = e;
                                index2 = k;
                                y= dist;
                                error_min = error;
                                min_dist = dist;
                            }
                        }
                    }
                }
                src[i] = {src[i][index1], src[i][index2]};
            }
            if(y > 0)
                dst.push_back(cv::Point(src[0][0]+i,y));
            //src[i].push_back(y);
        }
    }
    void print(const vvi &src){
        std::cout << "Line start: " << src[0][0] << std::endl;
        for(uint i = 1; i < src.size();i++){
            std::cout << "[" << i+src[0][0] << "]: ";
            for(uint k = 0; k < src[i].size();k++){
                std::cout << src[i][k] <<", ";
            }
            std::cout << std::endl;
        }
    }

}