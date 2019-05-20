#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <math.h>

#include <vector>
#include <deque>
#include <queue>

#include <PLineD.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <inspec_msg/line2d.h>
#include <inspec_msg/line2d_array.h>

#include <inspec_lib/Math.hpp>
#include <inspec_lib/settings/ReadSettings.hpp>

#include "Matcher.hpp"
#include "VanishingPointFilter.hpp"
#include "ProximityFiltering.hpp"
#include "ThicknessEstimator.hpp"


#define lineSeg std::vector< std::vector<cv::Point> >

#define MATCHER_LINE_MAX_ERROR 30
#define MATCHER_NO_MATCH_COST 30

#define DEBUG_PROXIMITY_FILTER_ false


using namespace std;

settings::Image_processing_node setting_node;
settings::PLineD setting_PLineD;
settings::Camera setting_camera;

typedef math::mathLine2d mathLine;

struct lineComparetor {
    bool operator() (vector<cv::Point> i,vector<cv::Point> j) { 
        return (i.size()>j.size());
    }
} lineComp;


// ###### ROS Variables #########
ros::NodeHandle* nh;
ros::Subscriber estimate_sub;
ros::Subscriber image_sub;
ros::Publisher line_pub;
ros::Publisher gotImage_pub;

// ###### Node Variables #########
cv::Mat img;
vector<mathLine> currentLines;
deque<inspec_msg::line2d_array> lineEstimates;
size_t img_num;
ros::Time image_time;
size_t estimate_num;
bool gotImage = false;

// ############## DEBUG ALGORITHMS ########################
void ShowImage(const string &name, const cv::Mat &img, int x = 50, int y = 50 ){
    cv::namedWindow(name,cv::WINDOW_NORMAL);
    cv::moveWindow(name,x,y);
    cv::resizeWindow(name, 600,500);
    cv::imshow(name, img);
}

// ############### PLineD #################################
lineSeg PLineD_full(cv::Mat &src){
    
    //########## Find Contours #############
    cv::Mat src_gray;
    lineSeg contours;

    cvtColor(src,src_gray,CV_BGR2GRAY);                     //Make Image gray
    PLineD::contoursCanny(src_gray,contours                 //Find Contours
                    ,setting_PLineD.canny_filter_size       //Gausian & sobal filter kernal size
                    ,setting_PLineD.canny_treshold_low      //Canny lower treshold limit
                    ,setting_PLineD.canny_treshold_high);   //Canny Upper treshold limit            
    
    //########### Cut segments ###################
    lineSeg segments; 

    PLineD::cutSeg(contours,segments                        //Cuts the conturs into more strait lines
                    ,setting_PLineD.segcut_min_length       //Minimum pixel on a line that can be cut up
                    ,setting_PLineD.segcut_max_angle        //Maximum angle of the line before it is cut
                    ,setting_PLineD.segcut_step_size);      //angle defined between Pixel(k-step),Pixel(k),Pixel(k+step)

    //########### Covariance filter ##############
    lineSeg covS;
    PLineD::segCov(segments,covS                            //Remove linesegments where the width to length ratio is bad
                    ,setting_PLineD.covariance_ratio);      //Minimum Width to Length ratio of line segment

    // ############ Grouping and Size filter #############
    lineSeg grpS;
    std::vector<cv::Point> angelVector;                     //Used to contain thedirection vector of line segments

    PLineD::groupSeg(covS,grpS,angelVector                  //Find and Combine line segments on same trajectory, and filter small lines away
                    ,setting_PLineD.group_min_end_length    //Minimum number of pixel a line must consist of at the end of the algorithm
                    ,setting_PLineD.group_min_start_length  //Minimum number of pixel a line must consist of to be base for extension.
                    ,setting_PLineD.group_max_angle_dif     //Maximum angel between lines
                    ,setting_PLineD.group_max_line_dist);   //max parallel distance between line and midt point of other line
    
    // ############ parallel lines ########
    lineSeg parallelLines;
    if(setting_PLineD.Parrallel_active){
        PLineD::parallelSeg(grpS, angelVector               //Finds the largest group of parallel lines
                    , parallelLines                         //Where to Store the Data
                    ,setting_PLineD.Parralel_max_angle);    //Maximum angle difference to be considered parallel
    }else{
        parallelLines = grpS;
    }
    //
    //Show Results
    if(setting_PLineD.debug){
        cv::Mat out1(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat out2(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat out3(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat out4(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat out5(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar(0,0,0));

        PLineD::printContours(out1, contours);
        PLineD::printContours(out2, segments);
        PLineD::printContours(out3, covS);
        PLineD::printContours(out4, grpS);
        if(setting_PLineD.Parrallel_active) PLineD::printContours(out5, parallelLines);
        cv::imshow("Conturs",out1);
        cv::imshow("SegCut",out2);
        cv::imshow("CovFil",out3);
        cv::imshow("GroupSeg",out4);
        if(setting_PLineD.Parrallel_active) cv::imshow("ParLines",out5);
        cv::waitKey(1);
    }

    return parallelLines;
}

// ############### ROS FUNCTIONS #########################

void image_handler(sensor_msgs::Image msg){
    if(!gotImage && ros::Time::now().toSec()-image_time.toSec() > 0.5){
        if(setting_node.debug) cout << "Image Num: " << msg.header.seq << endl;
        img_num++;
        image_time = msg.header.stamp;

        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        img = cv_ptr->image;
        //cv::resize(cv_ptr->image,img,cv::Size(1920,1080));      //Resize To 1920x1080
        gotImage = true;

        if(setting_node.show_incomming_image) {
            cv::imshow("TestImage",img);
            cv::waitKey(1);
        }
    }
}
void estimate_handler(inspec_msg::line2d_array msg){
    cout << "Recived Estimated Lines: " << msg.header.seq << " Size: "<< msg.lines.size() << endl;
    lineEstimates.push_back(msg);
    
    if(msg.header.seq != img_num){
        cerr << "Estimate out of sync" << endl;
    }
}
void PublishLinesToRos(vector<mathLine> lines){
    inspec_msg::line2d_array msg;
    inspec_msg::head h;
    h.seq = img_num;
    h.stamp = ros::Time::now();
    msg.header = h;
    for(int i = 0; i < lines.size(); i++){
        inspec_msg::line2d msg_line = convert::mathLine2ros(lines[i]);
        msg.lines.push_back(msg_line);
    }
    line_pub.publish(msg);
}
void PublishLinesToRos(vector<inspec_msg::line2d> &lines){
    inspec_msg::line2d_array msg;
    inspec_msg::head h;
    h.seq = img_num;
    h.stamp = ros::Time::now();
    msg.header = h;
    msg.lines = lines;
    line_pub.publish(msg);
}
void syncEstimateLines(){
    uint spins = 1000;

    if(!lineEstimates.empty()){
        while(lineEstimates.front().header.seq != img_num){
            if(setting_node.debug) cout << "Removing: " << lineEstimates.front().header.seq << endl;
            lineEstimates.pop_front();
            if(lineEstimates.empty()) break;
        }
    }
    while(lineEstimates.empty() && spins != 0){
        ros::spinOnce();
        spins--;
        if(setting_node.debug) if(spins == 0) cout << "Ran out of spins" << endl;
        if(!lineEstimates.empty()){
            if(setting_node.debug) cout << "Found Estimate - spins left: "<< spins << endl;
        } 
    }
    if(lineEstimates.empty()){
        if(setting_node.debug) cout << "No Line Estimates Available" << endl;
        lineEstimates.push_back(convert::line2d_array_construct());
        if(setting_node.debug) cout << "size: " << lineEstimates.size() << endl;
        return;
    }

    if(lineEstimates.size() > 1){
        if(setting_node.debug )cout << "More then one est" << endl;
        if(setting_node.debug) cout << "seq: " << lineEstimates.front().header.seq << endl;
        if(lineEstimates.front().header.seq > img_num){
            cerr << "Houston, We have a Problem" <<endl;
        }else{
            while(lineEstimates.front().header.seq < img_num){
                lineEstimates.pop_front();
                if(lineEstimates.empty()){
                    if(setting_node.debug) cout << "No Line Estimates" << endl;
                    lineEstimates.push_back(convert::line2d_array_construct());
                    break;
                }
            }
        }
    }else{
        if(setting_node.debug) cout << "One Estimate available " << endl;
        if(lineEstimates.front().header.seq != img_num){
            lineEstimates.front() = convert::line2d_array_construct(); 
        }
    }
    if(setting_node.debug) cout << "Estimated lines front is: " << lineEstimates.front().header.seq << endl;
}
void PublishGotImage(){
    inspec_msg::head h;
    h.seq = img_num;
    h.stamp = image_time;
    gotImage_pub.publish(h);
}

//################ MAIN ##################################
int main(int argc, char* argv[]){
    // ############## Start Ros ################
    ros::init(argc,argv,"Image_Processing");
    nh = new ros::NodeHandle();

    
    settings::read(setting_node);
    settings::read(setting_PLineD);
    settings::read(setting_camera);

    estimate_sub = nh->subscribe("/inspec/daq/Estimator/lines2d",10,estimate_handler);
    image_sub = nh->subscribe(setting_node.Image_topic.c_str(),1,image_handler);
    line_pub = nh->advertise<inspec_msg::line2d_array>("/inspec/daq/linedetector/lines2d",1);
    gotImage_pub = nh->advertise<inspec_msg::head>("/inspec/daq/linedetector/gotImage",1);

    // ############## Setup Output windows ##############
    cv::Mat BLACK(600, 600, CV_8UC3, cv::Scalar(0,0,0)); 
    if(setting_PLineD.debug){
        ShowImage("TestImage",BLACK ,20,20);
        ShowImage("Conturs",BLACK,610,20);
        ShowImage("SegCut",BLACK,1210,20);
        ShowImage("CovFil",BLACK,20,610);
        ShowImage("GroupSeg",BLACK,610,610);
        if(setting_PLineD.Parrallel_active) ShowImage("ParLines",BLACK,1210,610);
        ShowImage("PLineD",BLACK,1210,610);
    }else{
        if(setting_node.show_incomming_image) ShowImage("TestImage",BLACK);
        if(setting_node.show_final_image) ShowImage("PLineD",BLACK,50,600);
    }
    if(DEBUG_PROXIMITY_FILTER_){
        ShowImage("Proximity",BLACK,610,610);
    }

    
    cv::waitKey(1);
    // ############## Initialize Variables #############
    Matcher::LINE_MAX_ERROR = MATCHER_LINE_MAX_ERROR;
    Matcher::NO_MATCH_COST = MATCHER_NO_MATCH_COST;
    Prox::DEBUG_PROXIMITY_FILTER = DEBUG_PROXIMITY_FILTER_;
    int loop_num = -1;
    while(ros::ok()){

        //############## Reset and Wait for Data #####################
        currentLines.clear();
        gotImage = false;
        while(!gotImage && ros::ok()){
            ros::spinOnce();
        }
        if(!ros::ok()) break;
        PublishGotImage();

        loop_num++;
        // ############# Pline D #####################################
        if(setting_node.debug) {
            if(setting_node.debug) cout << endl << endl << "###########################################" << endl;
            if(setting_node.debug) cout << "Start Work Flow Image: " << img_num << endl;
        }

        lineSeg lines = PLineD_full(img);
        if(setting_node.debug) cout << "PlineD Done found " << lines.size() << " Lines" << endl;

        if(!lines.empty()){
            //############## Proximity Filtering ############################
            if(DEBUG_PROXIMITY_FILTER_){
                cv::Mat out1(img.rows, img.cols, CV_8UC3, cv::Scalar(0,0,0));
                Prox::filter(lines,out1);
                cv::imshow("Proximity",out1);
            }else{
                Prox::filter(lines);
                
            }
            if(setting_node.debug) cout << "Proximity Filter Lines Found: " << lines.size() << endl;

            // ############### Ready Data for Matching #######################
            for(uint i = 0; i < lines.size(); i++){
                mathLine l = math::leastSquareRegression(lines[i],img.size());
                //if(abs(l.a) < 2) // For Testing Indoor TODO
                currentLines.push_back(l);
            }
            // ############# Thickness Est ###############################
            /*ThickEst::vvi pLines;
            ThickEst::findParallelPixels(lines[0],currentLines[0],pLines);
            //ThickEst::print(pLines);
            ThickEst::vpf lineThick;
            ThickEst::findThickness(pLines,lineThick);
            mathLine Thickness = math::leastSquareRegression(lineThick,img.size(),false);
            cout << Thickness << endl;
            //ThickEst::print(pLines);*/
            // ############# Vannishing point Filter #####################
            if(setting_node.debug) cout << "Doing Vanishing point Filter" << endl;

            vector<mathLine> VPFiltered_lines;
            VP::filterLines(currentLines,VPFiltered_lines,50,300);
            currentLines = VPFiltered_lines;
            if(setting_node.debug) cout << "VPFilter found: " << currentLines.size() << endl;

            // ################ Line Matching ########################
            
            if(setting_node.debug) cout << "Line Mathcher" << endl;
            syncEstimateLines();
            vector<inspec_msg::line2d> matched_lines; 
            Matcher::matchingAlgorithm(matched_lines,currentLines,lineEstimates.front().lines);
            
            // ################ Finalize #############################
            if(setting_node.debug) cout << "Published Lines To Ros" << endl << endl;
            PublishLinesToRos(matched_lines);
            //################# DEBUG ################################
            if(setting_node.show_final_image){

                cv::Mat out(img.rows, img.cols, CV_8UC3, cv::Scalar(0,0,0));

                if(setting_node.debug) cout << "Drawing Found Lines: " << lines.size() << endl;
                //PLineD::printContours(out, lines);

                map<uint,bool> drawn;
                for(uint i = 0; i < matched_lines.size(); i++){
                    if(matched_lines[i].id != 0){
                        drawn[matched_lines[i].id] = true;
                        math::drawMathLine(out,convert::ros2mathLine(matched_lines[i]),cv::Scalar(0,255,0),"Match: "+ to_string(matched_lines[i].id),cv::Scalar(255,0,0));
                    }else{
                        math::drawMathLine(out,convert::ros2mathLine(matched_lines[i]),cv::Scalar(255,0,255), "Line: " + to_string(i));

                    }
                    
                }
                if(setting_node.debug) cout << "Drawing EST Lines: " << lineEstimates.size() << endl;

                if(!lineEstimates.empty()){
                    for(inspec_msg::line2d line: lineEstimates.front().lines){
                        if(!drawn[uint(line.id)]){
                            math::drawMathLine(out,convert::ros2mathLine(line),cv::Scalar(255,255,0),"Line: " + to_string(line.id));
                        }else{
                            math::drawMathLine(out,convert::ros2mathLine(line),cv::Scalar(0,255,230),"Match: "+ to_string(line.id),cv::Scalar(255,0,0));
                        }
                    }
                }
                //imwrite("LineMatch"+to_string(loop_num)+".jpg", out );

                cv::imshow("PLineD",out);
            }
            if(!setting_node.press_to_continue){ 
                cv::waitKey(1);
            }else{
                cv::waitKey(0);
            }
        }
        
    }

    return 0;
}
