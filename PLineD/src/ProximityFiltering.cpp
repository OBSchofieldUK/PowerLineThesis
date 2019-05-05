#include "ProximityFiltering.hpp"

namespace{
    struct lineComparetor {
        bool operator() (std::vector<cv::Point> i,std::vector<cv::Point> j) { 
            return (i.size()>j.size());
        }
    } lineComp;
}

namespace Prox{
    bool DEBUG_PROXIMITY_FILTER = false;
    
    void correctBBox(cv::RotatedRect &BBox){
        if(BBox.size.width < BBox.size.height){
            double holder = BBox.size.width;
            BBox.size.width = BBox.size.height;
            BBox.size.height = holder;
            BBox.angle += 90;
            if(BBox.angle> 180) BBox.angle -=360;
        }
    }
    cv::Point findBoxEnd(cv::RotatedRect BBox,uint side){
        int sign;
        if(side == RIGHT) sign = 1;
        else if (side == LEFT) sign = -1;
        else throw "NOT a valid side";

        double length,angle;
        if(BBox.size.width > BBox.size.height){ 
            angle = math::deg2rad(BBox.angle);
            length = BBox.size.width/2;
        }else{
            angle = math::deg2rad(BBox.angle +90);
            length = BBox.size.height/2;
        }
        cv::Point rightSide = BBox.center;
        rightSide.x += sign*length*cos(angle);
        rightSide.y += sign*length*sin(angle);
        return rightSide;
    }
    void findBoxes(lineSeg& src, std::vector<cv::RotatedRect> &dst, cv::Mat DEBUG_img){
        std::sort(src.begin(),src.end(),lineComp);
        dst = std::vector<cv::RotatedRect>(src.size());
        for( size_t i = 0; i < src.size(); i++ ){ 
            dst[i] = cv::minAreaRect( cv::Mat(src[i]) );
            correctBBox(dst[i]);
        }
        if(!DEBUG_img.empty()){
            for( size_t i = 0; i < src.size(); i++ ){
                cv::putText(DEBUG_img, "Line " + std::to_string(i),dst[i].center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255),2);
                math::mathLine2d l1 = math::constructMathLine(dst[i].center,math::deg2rad(dst[i].angle), &DEBUG_img);
                math::drawMathLine(DEBUG_img,l1,cv::Scalar(0,0,255));
                cv::circle(DEBUG_img,findBoxEnd(dst[i],RIGHT),12,cv::Scalar(255,255,0),3);
                cv::circle(DEBUG_img,findBoxEnd(dst[i],LEFT),12,cv::Scalar(255,0,255),3);
            }
        }

    }
    bool isExtensionLine(const boxLine &main, const boxLine &secondary, math::mathLine2d &Lmain, double &distReturn){
        bool toTheLeft=false;
        bool toTheRight=false;

        if(main.right.x < secondary.left.x){
            toTheRight = true;
            if(DEBUG_PROXIMITY_FILTER)
                std::cout << "     To the Right" << std::endl;
        }else if(main.left.x > secondary.right.x){
            toTheLeft = true;
            if(DEBUG_PROXIMITY_FILTER)
                std::cout << "     To the Left" << std::endl;
        }
        bool isExtension;
        if(toTheLeft){
            distReturn = math::distance(Lmain,secondary.right);
            isExtension = true;
        }else if(toTheRight){
            distReturn = math::distance(Lmain,secondary.left);
            isExtension = true;
        }else{
            distReturn = math::distance(Lmain,secondary.left)+math::distance(Lmain,secondary.right);
            isExtension = false;
        }
        return isExtension;
    }
    std::pair<uint,uint> larger(lineSeg &Lines,uint i1, uint i2){
        if(Lines[i1].size()> Lines[i2].size()){
            return std::pair<uint,uint>(i1,i2);
        }else{
            return std::pair<uint,uint>(i2,i1);
        }
    }
    void findLines2Filter(std::vector<cv::RotatedRect> &src, std::vector<uint> &remove, std::vector<std::pair<uint,uint> > &gather,double EXTENSION_MAX_ANGLE, double PARRALEL_MAX_ANGLE, double ADDTION_MAX_DISTANCE, double EXTENTSION_MAX_DISTANCE, double CENTER2LINE_DISTANCE_MAX){
        std::vector<bool> dontConsider(src.size());
        for(uint i = 0; i < src.size(); i++){
            if(!dontConsider[i]){
                boxLine B1 = {findBoxEnd(src[i],LEFT), findBoxEnd(src[i],RIGHT)};
                math::mathLine2d l1 = math::constructMathLine(src[i].center,math::deg2rad(src[i].angle));

                for(uint k = i+1; k < src.size(); k++){
                    if(!dontConsider[k]){

                        double angleDif = abs(src[i].angle-src[k].angle);

                        if(angleDif< EXTENSION_MAX_ANGLE){
                            math::mathLine2d l2 = math::constructMathLine(src[k].center,math::deg2rad(src[k].angle));

                            double dlp = math::distance(l1,src[k].center);
                            double dlp2 = math::distance(l2,src[i].center);
                            if(dlp2<dlp) dlp = dlp2;

                            if(DEBUG_PROXIMITY_FILTER) std::cout << "Lines(" << i << ", " << k << ") D: " << dlp << " A: " <<angleDif<<  std::endl;

                            if(dlp< CENTER2LINE_DISTANCE_MAX){
                                boxLine B2 = {findBoxEnd(src[k],LEFT), findBoxEnd(src[k],RIGHT)};
                                double dist;
                                std::pair<uint,uint> largest(i,k);

                                if(isExtensionLine(B1,B2,l1,dist)){ // EXtension
                                    if(DEBUG_PROXIMITY_FILTER) std::cout << " ### EXTENSION ###" << std::endl;
                                    if(dist<EXTENTSION_MAX_DISTANCE){

                                        if(angleDif>PARRALEL_MAX_ANGLE){ // Curved Extension
                                            remove.push_back(largest.second);
                                            dontConsider[largest.second]=true;
                                            if(DEBUG_PROXIMITY_FILTER) std::cout << "     Remove: " << largest.second << std::endl;

                                        }else{ // straight Extension
                                            dontConsider[largest.second];
                                            gather.push_back(largest);
                                            if(DEBUG_PROXIMITY_FILTER) std::cout << "     combine: " << largest.first << ", " << largest.second << std::endl;
                                        }
                                    }else if(DEBUG_PROXIMITY_FILTER){
                                        std::cout << "No Extension" << std::endl;
                                    }   
                                }else if(dist<ADDTION_MAX_DISTANCE && angleDif < PARRALEL_MAX_ANGLE){ //adition
                                    if(DEBUG_PROXIMITY_FILTER){
                                        std::cout << " ### Addition ###" << std::endl;
                                        std::cout << "     combine: " << largest.first << ", " << largest.second << std::endl;
                                    }
                                    
                                    dontConsider[largest.second];
                                    gather.push_back(largest);
                                }else if(DEBUG_PROXIMITY_FILTER){
                                    bool isExt = isExtensionLine(B1,B2,l1,dist);
                                    std::cout << "    is Extension: " << isExt <<  std::endl;
                                    std::cout << "    d:            " << dist << std::endl;
                                }
                            } 
                        }
                    }
                }
            }
        }
    }
    void applyFilterChanges(lineSeg &lines, std::vector<uint> &remove, std::vector<std::pair<uint,uint> > &gather){
        std::vector<bool> dontRemove(lines.size());
        for(std::pair<uint,uint> &x: gather){
            lines[x.first].insert(lines[x.first].end(),lines[x.second].begin(),lines[x.second].end());
            lines[x.second].clear();
            dontRemove[x.second] = true;
            dontRemove[x.first]  = true;
        }
        for(uint i: remove){
            bool doRemove = !dontRemove[i];         
            if(doRemove){
                lines[i].clear();
            } 
        }
        sort(lines.begin(),lines.end(),lineComp);
        for(uint i = lines.size()-1; i < lines.size(); i--){
            if(!lines[i].empty()){
                lines.erase(lines.begin()+i+1,lines.end());
                break;
            }
        }
    }
    void filter(lineSeg& lines,cv::Mat DEBUG_img, double EXTENSION_MAX_ANGLE, double PARRALEL_MAX_ANGLE, double ADDTION_MAX_DISTANCE, double EXTENTSION_MAX_DISTANCE, double CENTER2LINE_DISTANCE_MAX){
        std::vector<cv::RotatedRect> BBox(lines.size());
        Prox::findBoxes(lines,BBox,DEBUG_img);   
        if(!DEBUG_img.empty()) PLineD::printContours(DEBUG_img,lines);

        std::vector<uint> remove;
        std::vector<std::pair<uint,uint> > gather;
        Prox::findLines2Filter(BBox,remove,gather);

        Prox::applyFilterChanges(lines,remove,gather);
    }

}
