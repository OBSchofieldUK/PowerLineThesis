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

#define MATCHING_LINE_MAX_ERROR 30
#define MATCHER_NO_MATCH_COST MATCHING_LINE_MAX_ERROR

#define DEBUG_PLINED false
#define DEBUG true

using namespace std;

struct mathLine{
    double a;
    double b;
};

class candidate{

 public:
    candidate():angle(0),offset(0),index(0),error(MATCHER_NO_MATCH_COST),valid(false){}
    candidate(double angle,double offset,double index):angle(angle),index(index),valid(true){
        this->offset = abs(offset);
        this->errorCal();
    }
    void errorCal(){
        this->error = ((this->offset/5))*(1+int(this->offset/100))+ this->angle*180/M_PI;
    }
    bool operator< (const candidate &rhs) const{
        return this->error < rhs.error;
    }
    double angle;
    double offset;
    double error;
    uint index;
    bool valid;
};

ostream& operator<<(ostream& os, const mathLine& dt){
    os << "y = " << dt.a << "\t * x + " << dt.b ;
    return os;
}
ostream& operator<<(ostream& os, const candidate& dt){
    os << "Canditate[" << dt.index << "] Angle(" << dt.angle*180/M_PI << ") Offset(" << dt.offset << ") Error(" << dt.error << ")";
    return os;
}

ros::NodeHandle* nh;
ros::Subscriber estimate_sub;
ros::Subscriber image_sub;
ros::Publisher line_pub;

cv::Mat img;
vector<mathLine> currentLines;
deque<inspec_msg::line2d_array> lineEstimates;
size_t img_num;
size_t estimate_num;
bool gotImage = false;
// ############### CONVERTERS #########################
mathLine ros2mathLine(inspec_msg::line2d line){
    mathLine ret;
    double t = line.x0/line.dx;
    ret.b =  line.y0-line.dy*t;
    ret.a = line.dy/line.dx;
    
    return ret;
}
inspec_msg::line2d mathLine2ros(mathLine mline, uint id = 0){
    inspec_msg::line2d msg_line;
    msg_line.dx = 1;
    msg_line.dy = mline.a;
    msg_line.x0 = 0;
    msg_line.y0 = mline.b;
    msg_line.id = id;
    return msg_line;
}
double rad2deg(const double &angle){
    return angle*180/M_PI;
}
double deg2rad(const double &angle){
    return angle*M_PI/180;
}
// ############## DEBUG ALGORITHMS ########################
void ShowImage(const string &name, const cv::Mat &img, int x = 50, int y = 50 ){
    cv::namedWindow(name,cv::WINDOW_NORMAL);
    cv::moveWindow(name,x,y);
    cv::resizeWindow(name, 600,500);
    cv::imshow(name, img);
}
void drawMathLine(cv::Mat &dst, mathLine line, cv::Scalar color = cv::Scalar(255,255,255),string text = "",cv::Scalar textColor = cv::Scalar(255,255,255)){
    int x_max = 1920;
    int x_min = -1;
    for(int x = 0; x< dst.cols; x++){
        double b = (-line.b+dst.rows/2)-(-line.a)*(dst.cols/2);
        double a = -line.a;
        int y = a*x+b;
        if(y > 0 && y < dst.rows){
            dst.at<cv::Vec3b>(cv::Point(x,y)) = {uchar(color[0]),uchar(color[1]),uchar(color[2])};
            if(x_min == -1) x_min = x;
            
        }else if(x_max == 1920 && x_min != -1){
            x_max=x-1;
        }
    }
    if(!text.empty() && x_min != -1){
        double b = (-line.b+dst.rows/2)-(-line.a)*(dst.cols/2);
        double a = -line.a;
        double x = x_min+(x_max-x_min) / (1080/b);
        int y = a*x+b;

        cv::putText(dst,text,cv::Point(x,y),cv::FONT_HERSHEY_SIMPLEX,1,textColor,2);
    }
}

// ############### OTHER #################################
mathLine leastSquareRegression(std::vector<cv::Point> aLine){ // Should be const ref
    int N = aLine.size();
    double y_sum, x_sum, xy_sum, xx_sum;
    for(int i = 0; i < N; i++){
        const double x = aLine[i].x-img.cols/2;
        const double y = -(aLine[i].y-img.rows/2);

        y_sum += y;
        x_sum += x;
        xy_sum += x*y;
        xx_sum += x*x;
    }

    mathLine ret;
    ret.a = (N*xy_sum-x_sum*y_sum)/(N*xx_sum-(x_sum*x_sum));
    ret.b = (y_sum-ret.a*x_sum)/N;
    return ret;
}
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

double vecAngle(const inspec_msg::line2d &l1, const inspec_msg::line2d &l2){
    double length1 = sqrt(pow(l1.dx,2)+pow(l1.dy,2));
    double length2 = sqrt(pow(l2.dx,2)+pow(l2.dy,2));
    double dot = l1.dx*l2.dx+l1.dy*l2.dy;
    double angle = dot/(length1*length2);
    return abs(acos(angle));
}

pair<vector<candidate>,double> match_loop(vector<vector<candidate>> &candList,int i,vector<bool> not_available,double solution_cost,vector<candidate> Solution, double &Max_solution_cost){
    if(i >= candList.size()){
        if(DEBUG){
            cout <<endl << "############ Solution ##########" << endl;
            for(auto cand: Solution) cout << cand << endl;
        }
        if(solution_cost < Max_solution_cost) Max_solution_cost = solution_cost;
        return pair<vector<candidate>,double>(Solution,solution_cost);
    }

    pair<vector<candidate>,double> bedst_solution;
    bedst_solution.second = 9999999;

    for(int k = 0; k < candList[i].size(); k++){
        if(solution_cost+candList[i][k].error < Max_solution_cost){
            if(!not_available[candList[i][k].index]){

                if(candList[i][k].valid) not_available[candList[i][k].index] = true;

                Solution[i] = candList[i][k];
                pair<vector<candidate>,double> tmp_solution = match_loop(   candList,
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

void matchingAlgorithm(vector<inspec_msg::line2d> &result, const vector<mathLine> &slots, const vector<inspec_msg::line2d> &sockets){
    vector<bool> matched(slots.size());
    if(!sockets.empty()){
        vector<vector<candidate>> candList(sockets.size());
        vector<vector<candidate>> curLine_candList(slots.size());


        // ############# Find Error and Prioritys between slots and sockets ###########################
        const int x_point = 1920/2;
        for(uint i = 0; i < sockets.size(); i++){
            for(uint j = 0; j < slots.size(); j++ ){
                mathLine socket = ros2mathLine(sockets[i]);
                candidate c(
                    vecAngle(sockets[i],mathLine2ros(slots[j])),
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
            cout <<endl << "##########" << " Socket: " << sockets[i].id << "########" << endl; 
            for(auto cand: candList[i]) cout << cand <<endl;
        }

        // ####################### Match The Lines #########################
        vector<candidate> solution(candList.size());
        double max_cost = 999999;
        pair<vector<candidate>,double> result_tmp = match_loop(candList,0,vector<bool>(slots.size()),0,solution,max_cost);
        for(uint i = 0; i < result_tmp.first.size(); i++){
            if(result_tmp.first[i].valid){
                int index = result_tmp.first[i].index;
                matched[index] = true;
                result.push_back(mathLine2ros(slots[index],sockets[i].id));
            }
        }


        if(DEBUG){
            cout << endl << "############# Results ###############" << endl;
            for(int i = 0; i < result_tmp.first.size(); i++){
                cout << result_tmp.first[i] << endl;
            }
        }
    }

    //################### Add unmatched Lines ##########################
    cout << "Matching " << "Tidying up" << endl;
    for(int i = 0; i < matched.size(); i++){
        if(!matched[i]){
            result.push_back(mathLine2ros(slots[i]));
        }
    }
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
    /*for(auto line: msg.lines){
        mathLine m = ros2mathLine(line);
    }*/
    
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
        inspec_msg::line2d msg_line = mathLine2ros(lines[i]);
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
        return;
    }
    if(lineEstimates.front().header.seq > img_num){
        cerr << "Houston, We have a Problem" <<endl;
    }else{
        while(lineEstimates.front().header.seq < img_num){
            if(lineEstimates.empty()){
                cout << "No Line Estimates" << endl;
                break;
            }
            lineEstimates.pop_front();
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
            currentLines.push_back(leastSquareRegression(lines[i]));
        }
        syncEstimateLines();
        // ################ Line Matching ########################
        cout << "Line Mathcher" << endl;
        
        vector<inspec_msg::line2d> matched_lines;
        if(lineEstimates.empty()){
            matchingAlgorithm(matched_lines,currentLines,vector<inspec_msg::line2d>() );
        }else{
            matchingAlgorithm(matched_lines,currentLines,lineEstimates.front().lines);
        }
        
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
                drawMathLine(out,ros2mathLine(matched_lines[i]),cv::Scalar(0,255,0),"Match: "+ to_string(matched_lines[i].id),cv::Scalar(255,0,0));
            }else{
                drawMathLine(out,ros2mathLine(matched_lines[i]),cv::Scalar(255,0,255), "Line: " + to_string(i));
            }
            
        }
        cout << "Drawing EST Lines" << endl;
        if(!lineEstimates.empty()){
            for(inspec_msg::line2d line: lineEstimates.front().lines){
                if(!drawn[line.id]){
                    drawMathLine(out,ros2mathLine(line),cv::Scalar(255,255,0),"Line: " + to_string(line.id));
                }else{
                    drawMathLine(out,ros2mathLine(line),cv::Scalar(0,255,230),"Match: "+ to_string(line.id),cv::Scalar(255,0,0));
                }
            }
        }
        imwrite( "./LineMatch"+to_string(loop_num)+".jpg", out );

        cv::imshow("PLineD",out);
        cv::waitKey(1);
        
    }

    return 0;
}
