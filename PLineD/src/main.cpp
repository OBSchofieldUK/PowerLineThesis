#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
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


ros::NodeHandle* nh;
ros::Subscriber estimate_sub;
ros::Subscriber image_sub;
ros::Publisher line_pub;

cv::Mat img;
vector<inspec_msg::line2d> lineEstimates;
size_t img_num;
bool gotImage = false;
// ############## DEBUG ALGORITHMS ########################
void ShowImage(const string &name, const cv::Mat &img, int x = 50, int y = 50 ){
    cv::namedWindow(name,cv::WINDOW_NORMAL);
    cv::moveWindow(name,x,y);
    cv::resizeWindow(name, 600,500);
    cv::imshow(name, img);
}
void drawMathLine(cv::Mat &dst, mathLine line, cv::Scalar color = cv::Scalar(255,255,255)){
    vector<cv::Mat> channels;
    cv::split(dst,channels);
    for(int i = 0; i< dst.cols; i++){
        int y = line.a*i+line.b;
        if(y > 0 && y < dst.rows){
            dst.at<cv::Vec3b>(cv::Point(i,y)) = {uchar(color[0]),uchar(color[1]),uchar(color[2])};
        }
    }
}

// ############### OTHER #################################
mathLine leastSquareRegression(const std::vector<cv::Point> &aLine){
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
    cv::resize(src_gray,src_gray,cv::Size(1920,1080));      //Resize To 1920x1080
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
    img = cv_ptr->image;
    gotImage = true;
    cv::imshow("TestImage",img);
    cv::waitKey(1);
}
void estimate_handler(inspec_msg::line2d_array msg){
    cout << "Recived Estimated Lines: " << msg.header.seq << endl;
    lineEstimates = msg.lines;

    if(msg.header.seq != img_num){
        cerr << "Estimate out of sync" << endl;
    }
}
void PublishLinesToRos(lineSeg lines){
    inspec_msg::line2d_array msg;
    inspec_msg::head h;
    h.seq = img_num;
    h.stamp = ros::Time::now();
    msg.header = h;
    for(int i = 0; i < lines.size(); i++){
        mathLine mline = leastSquareRegression(lines[i]);
        inspec_msg::line2d msg_line;
        msg_line.dx = 1;
        msg_line.dy = mline.a;
        msg_line.x0 = 0;
        msg_line.y0 = mline.b;
        msg.lines.push_back(msg_line);
    }
    line_pub.publish(msg);
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
    ShowImage("PLineD",BLACK,50,650);
    //ShowImage("PLineD2",BLACK,50,650);
    
    while(ros::ok()){
        while(!gotImage && ros::ok()){
            ros::spinOnce();
        }
        lineSeg lines = PLineD_full(img,false);
        PublishLinesToRos(lines);      
        gotImage = false;
        
        cv::Mat out(1080, 1920, CV_8UC3, cv::Scalar(0,0,0));
        PLineD::printContours(out, lines);
        /*for(int i = 0; i < lines.size(); i++){
            mathLine Line = leastSquareRegression(lines[i]);
            drawMathLine(out,Line);
        }*/
        
        /*for(inspec_msg::line2d line: lineEstimates){
            drawMathLine(out,ros2mathLine(line),cv::Scalar(0,255,0));
        }*/
        //PLineD::printContours(out, lines);
        //cout << "Parrallel Lines: " << parallelLines.size() << endl;
        cv::imshow("PLineD2",out);
        cv::waitKey(1);
    }

    return 0;
}
