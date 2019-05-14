#include <iostream>

#include <chrono>
#include <thread>

#include <inspec_lib/Math.hpp>
#include <inspec_lib/settings/ReadSettings.hpp>

#include <ros/ros.h>
#include <inspec_msg/head.h>
uint img_num;
ros::Publisher gotImage_pub;

void PublishGotImage(){
    inspec_msg::head h;
    h.seq = img_num++;
    h.stamp = ros::Time::now();
    gotImage_pub.publish(h);
}

int main(int argc, char* argv[]){
    // ############## Start Ros ################
    ros::init(argc,argv,"Image_Processing");
    ros::NodeHandle *nh = new ros::NodeHandle();
    //estimate_sub = nh->subscribe("/inspec/daq/Estimator/lines2d",1,estimate_handler);
    //image_sub = nh->subscribe("/webcam/image_raw",1,image_handler);
    //line_pub = nh->advertise<inspec_msg::line2d_array>("/inspec/daq/linedetector/lines2d",1);
    gotImage_pub = nh->advertise<inspec_msg::head>("/inspec/daq/linedetector/gotImage", 1);

    while(ros::ok()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        PublishGotImage();
    }

    return 0;
}