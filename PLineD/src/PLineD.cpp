#include "PLineD.hpp"
#include <math.h>
#include <string>

cv::RNG rng(cv::getTickCount());
namespace {
struct lineSeg_less_than_key{
    inline bool operator() (const std::vector<cv::Point>& struct1, const std::vector<cv::Point>& struct2)
    {
        return (struct1.size() > struct2.size());
    }
    void ShowImage(const std::string &name, const cv::Mat &img, int x = 50, int y = 50 ){
        cv::namedWindow(name,cv::WINDOW_NORMAL);
        cv::moveWindow(name,x,y);
        cv::resizeWindow(name, 600,500);

    }
};
double diffAngle(std::vector<cv::Point> vec1, std::vector<cv::Point> vec2){
    cv::Point V1 = vec1.front()-vec1.back();
    cv::Point V2 = vec2.front()-vec2.back();
    double A= acos(V1.dot(V2)  /  (sqrt(   V1.dot(V1)   )*sqrt(  V2.dot(V2)  )));
    if(A > M_PI/2) A = M_PI-A;
    return A;
}
cv::Mat calcVecCovar(const std::vector<cv::Point> &inRow){
    double varXY=0, varXX= 0, varYY =0;
    cv::Mat coVarMatx = cv::Mat(2,2,CV_64F);

    double meanX = 0, meanY = 0;
    for (size_t i = 0; i < inRow.size(); i++){
        meanX += inRow[i].x;
        meanY += inRow[i].y;
    }
    meanX /= inRow.size();
    meanY /= inRow.size();
    
    for (size_t i = 0; i < inRow.size(); i++) {
        varXX += ((inRow[i].x-meanX) * (inRow[i].x-meanX));
        varXY += ((inRow[i].x-meanX) * (inRow[i].y-meanY));
        varYY += ((inRow[i].y-meanY) * (inRow[i].y-meanY));
    }
    varXX /= inRow.size()-1;
    varXY /= inRow.size()-1;
    varYY /= inRow.size()-1;
    coVarMatx.at<double>(0,0) = varXX;
    coVarMatx.at<double>(0,1) = varXY;
    coVarMatx.at<double>(1,0) = varXY;
    coVarMatx.at<double>(1,1) = varYY;
    return coVarMatx;
}
double calcDiffAngle(cv::Point V1, cv::Point V2){
    double A= acos(V1.dot(V2)  /  (sqrt(   V1.dot(V1)   )*sqrt(  V2.dot(V2)  )));
    if(A > M_PI/2) A = M_PI-A;
    return A;
}
}

namespace PLineD{
void cutSeg(const lineSeg &src,lineSeg &dst,const size_t minLen ,double angle,const size_t step){
    double A;
    cv::Point V1,V2;
    size_t last,i,z;

    angle *= (M_PI/180);
    for(i = 0; i< src.size(); i++ ){
        if(src[i].size()>minLen){
            if(dst.size() >0 && dst.back().size() == 1) dst.pop_back();
            dst.push_back(std::vector<cv::Point>());
            last = 0;
            for(z = 0; z < src[i].size(); z++ ){
                dst.back().push_back(src[i][z]);
                if(z-step>=last && z+step <src[i].size()){
                    V1 = src[i][z] -src[i][z-step];
                    V2 = src[i][z+step] -src[i][z];
                    A = acos(V1.dot(V2)  /  (sqrt(   V1.dot(V1)   )*sqrt(  V2.dot(V2)  )));
                    if(A>angle){
                        last = z;
                        if(dst.back().size() < minLen) dst.pop_back();
                        dst.push_back(std::vector<cv::Point>());

                    }
                }
            }
        }
    }
}
void contoursCanny(const cv::Mat &src,lineSeg &contours,uint8_t kernel_size,uint8_t low_treshold, uint8_t high_treshold){
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat img;
    cv::blur(src,img,cv::Size(kernel_size,kernel_size));
    Canny(img, img, low_treshold, high_treshold, kernel_size);
    findContours( img, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
}
void printContour(cv::Mat &dst, const std::vector< cv::Point> &contour, cv::Scalar color){
    for( size_t z = 0; z < contour.size(); z++ ){
            cv::Point p = contour[z];
            if(p.x >= 0 && p.x < dst.cols && p.y>=0 && p.y <dst.rows){
                dst.at<cv::Vec3b>(contour[z]) = {uchar(color[0]), uchar(color[1]), uchar(color[2])};
            }else{
                std::cerr << "Index out of bounds: P(" << p.x << ", " << p.y << ") - Bounds(" << dst.cols << ", " << dst.rows << ")" << std::endl;
            }
        }
}
void printContours(cv::Mat &dst, const lineSeg &contours){
    for( size_t i = 0; i< contours.size(); i++ ){
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        printContour(dst,contours[i],color);
    }
}
void drawPoints(cv::Mat &dst,const std::vector<cv::Point> &pointList, const cv::Scalar color){
    for(size_t i = 0; i < pointList.size(); i++){
        dst.at<cv::Vec3b>(pointList[i]) = {uchar(color[0]), uchar(color[1]), uchar(color[2])};
    }

}
void groupSeg(lineSeg src, lineSeg &dst,std::vector<cv::Point> &dst_angels,const size_t miP, const size_t miL, double maA, const size_t dS){
    if(src.size() == 0) return;
    maA*=(M_PI/180);
    std::sort(src.begin(), src.end(), lineSeg_less_than_key());
    dst.push_back(src.front());
    dst_angels.push_back(src.front().front()-src.front().back());
    
    for(size_t i = 0; i < src.size(); i++){
        if(src[i].size()>miL){
            for(size_t z = i+1; z < src.size(); z++){
                if(diffAngle(src[i],src[z]) < maA){
                    cv::Point V1 = src[i].back()-src[i].front();
                    cv::Point V2 = src[z][src[z].size()/2] - src[i].back();
                    double dist = std::abs(V1.cross(V2))/sqrt(V1.dot(V1));
                    if(dist < dS){
                        dst.back().insert(dst.back().end(),src[z].begin(),src[z].end());
                        src.erase(src.begin()+long(z));
                        z--;
                    }
                }
            }
        }else{
            break;
        }
        if(src.size()>i+1){
            if(dst.back().size()>miP){
                dst.push_back(src[i+1]);
                dst_angels.push_back(src[i+1].front()-src[i+1].back());
            }else{
                dst.back().clear();
                dst_angels.back() =src[i+1].front()-src[i+1].back();
            }
        }
    }
    if(dst.back().size() < miP){
        dst.pop_back();
        dst_angels.pop_back();
    }
}
void segCov(const lineSeg &cutS,lineSeg &reducedSeg, int ratio){
    for (size_t i = 0; i < cutS.size(); i++) {
        cv::Mat COV, eigenVals;
        if (cutS[i].size() > 1){  
            COV = calcVecCovar(cutS[i]);
            cv::eigen(COV, eigenVals);
            double lambda1 = eigenVals.at<double>(0,0);
            double lambda2 = eigenVals.at<double>(0,1);
            if ((lambda1/lambda2) > ratio ){
                reducedSeg.push_back(cutS[i]);
            }
        }
    }
}
void parallelSeg(const lineSeg &inPts, const std::vector<cv::Point> &anglePts, lineSeg &paraSegments, double tAngle, size_t minPPL){
    lineSeg tmpSegments;
    for (size_t i = 0; i < inPts.size(); i++) {
    // cout << i << ": " << endl << endl;

    for (size_t j = 0; j < inPts.size(); j++) {
        float diffAngle = calcDiffAngle(anglePts[i], anglePts[j]) * (180/M_PI);
        if (i != j){
        // cout << "i: " << i << " j: " << j << " AngleDif: " << diffAngle;
        if (diffAngle < tAngle){
            // cout << " pushed";
            tmpSegments.push_back(inPts[i]);
        }
        // cout << endl;
        }
        }
    }
}
}
