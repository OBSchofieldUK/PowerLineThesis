#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <string>
#include <math.h>

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

using namespace std;

struct mathLine{
    double a;
    double b;
};

ostream& operator<<(ostream& os, const mathLine& dt){
    os << "y = " << dt.a << "\t * x + " << dt.b ;
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

// ############## DEBUG ALGORITHMS ########################
void ShowImage(const string &name, const cv::Mat &img, int x = 50, int y = 50 ){
    cv::namedWindow(name,cv::WINDOW_NORMAL);
    cv::moveWindow(name,x,y);
    cv::resizeWindow(name, 600,500);
    cv::imshow(name, img);
}
void drawMathLine(cv::Mat &dst, mathLine line, cv::Scalar color = cv::Scalar(255,255,255)){
    for(int x = 0; x< dst.cols; x++){
        double b = (-line.b+dst.rows/2)-(-line.a)*(dst.cols/2);
        double a = -line.a;
        int y = a*x+b;
        if(y > 0 && y < dst.rows){
            dst.at<cv::Vec3b>(cv::Point(x,y)) = {uchar(color[0]),uchar(color[1]),uchar(color[2])};
        }
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
lineSeg PLineD_full(cv::Mat &src,bool DEBUG=false){
    
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
    if(DEBUG){
        cv::Mat out(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar(0,0,0));
        PLineD::printContours(out, parallelLines);
        cv::imshow("PLineD",out);
        cv::waitKey(1);
    }

    return parallelLines;
}
double rad2deg(const double &angle){
    return angle*180/M_PI;
}
double deg2rad(const double &angle){
    return angle*M_PI/180;
}
double angle(const inspec_msg::line2d &l1, const inspec_msg::line2d &l2){
    double length1 = sqrt(pow(l1.dx,2)+pow(l1.dy,2));
    double length2 = sqrt(pow(l2.dx,2)+pow(l2.dy,2));
    double dot = l1.dx*l2.dx+l1.dy*l2.dy;
    double angle = dot/(length1*length2);
    return abs(acos(angle));
}
// ############### ROS FUNCTIONS #########################
mathLine ros2mathLine(inspec_msg::line2d line){
    mathLine ret;
    double t = line.x0/line.dx;
    ret.b =  line.y0-line.dy*t;
    ret.a = line.dy/line.dx;
    
    return ret;
}
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
    for(auto line: msg.lines){
        mathLine m = ros2mathLine(line);
    }
    
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
        mathLine mline = lines[i];
        inspec_msg::line2d msg_line;
        msg_line.dx = 1;
        msg_line.dy = mline.a;
        msg_line.x0 = 0;
        msg_line.y0 = mline.b;
        //cout << "Sending Line: " << ros2mathLine(msg_line) << endl; // Code Doesn't work without this line duno why
        msg.lines.push_back(msg_line);
    }
    line_pub.publish(msg);
}
void syncEstimateLines(){
    ros::spinOnce();
    if(lineEstimates.empty()){
        cout << "No Line Estimates Available" << endl;
        return;
    }
    if(lineEstimates.front().header.seq > img_num){
        cerr << "Huston We have a Problem" <<endl;
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
    
    while(ros::ok()){
        currentLines.clear();
        while(!gotImage && ros::ok()){
            ros::spinOnce();
        }
        cout << endl << endl << "###########################################" << endl;
        cout << "Start Work Flow Image: " << img_num << endl;

        lineSeg lines = PLineD_full(img,false);
        cout << "PlineD Done found " << lines.size() << " Lines" << endl;
       
        for(int i = 0; i < lines.size(); i++){
            currentLines.push_back(leastSquareRegression(lines[i]));
        }
        syncEstimateLines();

        // ################ Line Matching ########################



        // ################ Finalize #############################
        PublishLinesToRos(currentLines);
        cout << "Published Lines To Ros" << endl << endl;
        
        //################# DEBUG ################################


        cv::Mat out(img.rows, img.cols, CV_8UC3, cv::Scalar(0,0,0));

        cout << "Drawing Found Lines" << endl;
        PLineD::printContours(out, lines);
        for(int i = 0; i < currentLines.size(); i++){
            drawMathLine(out,currentLines[i]);
        }
        cout << "Drawing EST Lines" << endl;
        if(!lineEstimates.empty()){
            for(inspec_msg::line2d line: lineEstimates.front().lines){
                drawMathLine(out,ros2mathLine(line),cv::Scalar(0,255,0));
            }
        }

        cv::imshow("PLineD",out);
        cv::waitKey(1);
        gotImage = false;
    }

    return 0;
}
