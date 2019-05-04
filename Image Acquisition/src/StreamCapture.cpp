#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char **argv){
    ros::init(argc,argv,"Airsim_images");
    ros::NodeHandle nh = ros::NodeHandle();
    cv::VideoCapture cap("http://admin:12345@10.42.0.72:14551/video_feed");
    cv::Mat img;
    bool succes = cap.read(img);

    if(succes){
        cv::imshow("Img",img);
        cv::waitKey(0);
    }else{
        cout << "Somthing Went wrong" << endl;
    }
    
    return 0;
}