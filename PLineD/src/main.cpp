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
#include "Matcher.hpp"


#define lineSeg std::vector< std::vector<cv::Point> >

#define IMAGE_CONTOURS_FILTER_KERNAL_SIZE 3u    //Must be uneven positive integer
#define IMAGE_CONTOURS_TRESHOLD_LOW 36u         //between 0 and 255
#define IMAGE_CONTOURS_TRESHOLD_HIGH 150u       //between 0 and 255 should be larger the low treshold
#define LINE_CUT_MIN_LENGTH 15u                 //number of pixel in line
#define LINE_CUT_MAX_ANGLE 30.0f                //Angle in deg
#define LINE_CUT_STEP_SIZE 5u                   //Array positions
#define LINE_COV_RATIO 600u                     //covariance eigen vector ratio
#define LINE_GROUP_MIN_START_LENGTH 100u        //length in pixel
#define LINE_GROUP_MIN_END_LENGTH 400u          //length in pixel
#define LINE_GROUP_MAX_ANGLE_DIF 5.0f           //Angle in deg
#define LINE_GROUP_MAX_LINE_DIST 10             //distance in pixel
#define LINE_PARALLEL_ACTIVE false              //activate for extra sorting if nessesary
#define LINE_PARALLEL_MAX_ANGLE 5.0f            //Angle in deg

#define MATCHER_LINE_MAX_ERROR 30
#define MATCHER_NO_MATCH_COST MATCHER_LINE_MAX_ERROR

#define DEBUG_PLINED false
#define DEBUG true

using namespace std;

typedef math::mathLine2d mathLine;

// ###### ROS Variables #########
ros::NodeHandle* nh;
ros::Subscriber estimate_sub;
ros::Subscriber image_sub;
ros::Publisher line_pub;

// ###### Node Variables #########
cv::Mat img;
vector<mathLine> currentLines;
deque<inspec_msg::line2d_array> lineEstimates;
size_t img_num;
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
                    ,IMAGE_CONTOURS_FILTER_KERNAL_SIZE      //Gausian & sobal filter kernal size
                    ,IMAGE_CONTOURS_TRESHOLD_LOW            //Canny lower treshold limit
                    ,IMAGE_CONTOURS_TRESHOLD_HIGH);         //Canny Upper treshold limit            
    
    //########### Cut segments ###################
    lineSeg segments; 

    PLineD::cutSeg(contours,segments                        //Cuts the conturs into more strait lines
                    ,LINE_CUT_MIN_LENGTH                    //Minimum pixel on a line that can be cut up
                    ,LINE_CUT_MAX_ANGLE                     //Maximum angle of the line before it is cut
                    ,LINE_CUT_STEP_SIZE);                   //angle defined between Pixel(k-step),Pixel(k),Pixel(k+step)

    //########### Covariance filter ##############
    lineSeg covS;
    PLineD::segCov(segments,covS                            //Remove linesegments where the width to length ratio is bad
                    ,LINE_COV_RATIO);                       //Minimum Width to Length ratio of line segment

    // ############ Grouping and Size filter #############
    lineSeg grpS;
    std::vector<cv::Point> angelVector;                     //Used to contain thedirection vector of line segments

    PLineD::groupSeg(covS,grpS,angelVector                  //Find and Combine line segments on same trajectory, and filter small lines away
                    ,LINE_GROUP_MIN_END_LENGTH              //Minimum number of pixel a line must consist of at the end of the algorithm
                    ,LINE_GROUP_MIN_START_LENGTH            //Minimum number of pixel a line must consist of to be base for extension.
                    ,LINE_GROUP_MAX_ANGLE_DIF               //Maximum angel between lines
                    ,LINE_GROUP_MAX_LINE_DIST);             //max parallel distance between line and midt point of other line
    
    // ############ parallel lines ########
    lineSeg parallelLines;
    if(LINE_PARALLEL_ACTIVE){
        PLineD::parallelSeg(grpS, angelVector               //Finds the largest group of parallel lines
                    , parallelLines                         //Where to Store the Data
                    ,LINE_PARALLEL_MAX_ANGLE);              //Maximum angle difference to be considered parallel
    }else{
        parallelLines = grpS;
    }
    //
    //Show Results
    if(DEBUG_PLINED){
        cv::Mat out(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar(0,0,0));
        PLineD::printContours(out, parallelLines);
        cv::imshow("PLineD",out);
        cv::waitKey(1);
    }

    return parallelLines;
}

// ############### ROS FUNCTIONS #########################

void image_handler(sensor_msgs::Image msg){
    //cv::Mat img = cv::Mat(msg.data);
    cout << "Image Num: " << msg.header.seq << endl;
    img_num = msg.header.seq;
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::resize(cv_ptr->image,img,cv::Size(1920,1080));      //Resize To 1920x1080
    gotImage = true;
    cv::imshow("TestImage",img);
    cv::waitKey(1);
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
        inspec_msg::line2d msg_line = converter::mathLine2ros(lines[i]);
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
    ros::spinOnce();
    if(lineEstimates.empty()){
        cout << "No Line Estimates Available" << endl;
        lineEstimates.push_back(converter::line2d_array_construct());
        cout << "size: " << lineEstimates.size() << endl;
        return;
    }
    if(lineEstimates.size() > 1){
        if(lineEstimates.front().header.seq > img_num){
            cerr << "Houston, We have a Problem" <<endl;
        }else{
            while(lineEstimates.front().header.seq < img_num){
                if(lineEstimates.empty()){
                    cout << "No Line Estimates" << endl;
                    lineEstimates.push_back(converter::line2d_array_construct());
                    break;
                }
                lineEstimates.pop_front();
            }
        }
    }else{
        if(lineEstimates.front().header.seq != img_num){
            lineEstimates.front() = converter::line2d_array_construct(); 
        }
    }
    cout << "Estimated lines front is: " << lineEstimates.front().header.seq << endl;
}

//################ MAIN ##################################
int main(int argc, char* argv[]){
    // ############## Start Ros ################
    ros::init(argc,argv,"plined");
    nh = new ros::NodeHandle();
    estimate_sub = nh->subscribe("/Estimator/lines2d",1,estimate_handler);
    image_sub = nh->subscribe("/webcam/image_raw",1,image_handler);
    line_pub = nh->advertise<inspec_msg::line2d_array>("/linedetector/lines2d",1);
    // ############## Setup Output windows ##############
    cv::Mat BLACK(600, 600, CV_8UC3, cv::Scalar(0,0,0)); 
    ShowImage("TestImage",BLACK);
    ShowImage("PLineD",BLACK,50,600);
    cv::waitKey(1);
    // ############## Initialize Variables #############
    Matcher::LINE_MAX_ERROR = MATCHER_LINE_MAX_ERROR;
    Matcher::NO_MATCH_COST = MATCHER_NO_MATCH_COST;
    int loop_num = -1;
    while(ros::ok()){

        //############## Reset and Wait for Data #####################
        currentLines.clear();
        while(!gotImage && ros::ok()){
            ros::spinOnce();
        }
        loop_num++;
        // ############# Pline D #####################################
        std::cout << endl << endl << "###########################################" << endl;
        cout << "Start Work Flow Image: " << img_num << endl;

        lineSeg lines = PLineD_full(img);
        cout << "PlineD Done found " << lines.size() << " Lines" << endl;
       

       // ############### Ready Data for Matching #######################
        for(uint i = 0; i < lines.size(); i++){
            currentLines.push_back(math::leastSquareRegression(lines[i],img.size()));
        }
        syncEstimateLines();
        // ################ Line Matching ########################
        cout << "Line Mathcher" << endl;
        
        vector<inspec_msg::line2d> matched_lines; 
        Matcher::matchingAlgorithm(matched_lines,currentLines,lineEstimates.front().lines);
        
        // ################ Finalize #############################
        cout << "Published Lines To Ros" << endl << endl;
        PublishLinesToRos(matched_lines);
        gotImage = false;
        //################# DEBUG ################################


        cv::Mat out(img.rows, img.cols, CV_8UC3, cv::Scalar(0,0,0));

        cout << "Drawing Found Lines: " << loop_num << endl;
        //PLineD::printContours(out, lines);

        map<int,bool> drawn;
        for(int i = 0; i < matched_lines.size(); i++){
            if(matched_lines[i].id != 0){
                drawn[matched_lines[i].id] = true;
                math::drawMathLine(out,converter::ros2mathLine(matched_lines[i]),cv::Scalar(0,255,0),"Match: "+ to_string(matched_lines[i].id),cv::Scalar(255,0,0));
            }else{
                math::drawMathLine(out,converter::ros2mathLine(matched_lines[i]),cv::Scalar(255,0,255), "Line: " + to_string(i));
            }
            
        }
        cout << "Drawing EST Lines" << endl;
        if(!lineEstimates.empty()){
            for(inspec_msg::line2d line: lineEstimates.front().lines){
                if(!drawn[line.id]){
                    math::drawMathLine(out,converter::ros2mathLine(line),cv::Scalar(255,255,0),"Line: " + to_string(line.id));
                }else{
                    math::drawMathLine(out,converter::ros2mathLine(line),cv::Scalar(0,255,230),"Match: "+ to_string(line.id),cv::Scalar(255,0,0));
                }
            }
        }
        imwrite("LineMatch"+to_string(loop_num)+".jpg", out );

        cv::imshow("PLineD",out);
        cv::waitKey(1);
        
    }

    return 0;
}
