#include "ThicknessEstimator.hpp"
namespace{
    cv::Point2f Rotate(cv::Point2f &src, double angle){
        cv::Point2f p;
        src.x = std::cos(angle)*src.x - std::sin(angle)*src.y;
        src.y = std::sin(angle)*src.x + std::cos(angle)*src.y;
        return p;
    }
    cv::Point2f Rotate(cv::Point &src, double angle){
        cv::Point2f p;
        p.x = std::cos(angle)*src.x - std::sin(angle)*src.y;
        p.y = std::sin(angle)*src.x + std::cos(angle)*src.y;
        return p;
    }
}


namespace ThickEst{
    void findParallelPixels(const vp &src,const math::mathLine2d &line, vvi &dst, const cv::Size &imgSize ){
        dst = vvi(imgSize.width);
        int first=imgSize.width,last= -imgSize.width;
        cv::Point left;
        left.x = -imgSize.width/2;
        left.y = line.a*left.x;
        Rotate(left,-std::atan(line.a));
        int minX = left.x-100;

        for(auto p: src){
            convert::img2realCoordOverride(p,imgSize);
            p.y-= line.b;
            cv::Point2f point = Rotate(p,-std::atan(line.a));
            point.x -= minX;
            bool notThere = true;
            
            for(uint i = 0; i < dst[point.x].size(); i++){
                if(dst[point.x][i] == point.y){
                    notThere = false;
                    break;
                }
            }
            if(notThere){
                int x = int(point.x);
                dst[x].push_back(point.y);
                if(dst[x].size()>1){
                    if(x < first){
                        first = x;
                    } 
                    if(x > last) {
                        last = x;
                    }
                }
            }
        }

        if(last < dst.size()-1)dst.erase(dst.begin()+last+1,dst.end());
        else if(last == dst.size()-1) dst.erase(dst.end());

        if(first > 1) dst.erase(dst.begin(),dst.begin()+first-1);
        else if(first == 1) dst.erase(dst.begin());

        dst.push_front({float(first-1+minX)});
    }
    float minDist(float point, const vi &options){
        float min = 5000;
        for(uint i = 0; i < options.size(); i++){
            float dist = std::abs(options[i]-point);
            if(dist < min){
                min = dist;
                if(dist == 0) break;
            }
        }
        return min;
    }
    void findThickness(vvi &src, vpf &dst){
        for(uint i = 1; i < src.size(); i++){
            float y = -1;
            if(src[i].size()== 2){
                y = std::abs(src[i][0]-src[i][1]);
                if(y < 2){ 
                    y = -1;
                    src[i] = {};
                }
            }else if(src[i].size() > 2){
                uint index1 =0 , index2 = 0;
                float error_min = -1;
                float min_dist = -1;
                for(uint e = 0; e < src[i].size(); e++){
                    for(uint k = e+1; k < src[i].size(); k++){
                        float dist = std::abs(src[i][e]-src[i][k]);
                        if(dist > 1){
                            float error = 0;
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
            if(y > 0){
                dst.push_back(cv::Point2f(src[0][0]+i,y));
            }
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