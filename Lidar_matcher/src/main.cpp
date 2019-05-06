
#include <ros/ros.h>

#include <inspec_msg/lidardat.h>
#include <inspec_msg/line2d_array.h>
#include <inspec_msg/line2d.h>
#include <inspec_msg/matched_lidar_data_array.h>
#include <inspec_msg/matched_lidar_data.h>

#include <inspec_lib/settings/SettingStructs.hpp>
#include <inspec_lib/settings/ReadSettings.hpp>

#include <deque>
#include <vector>

using namespace std;

ros::Publisher Matched_pub;
settings::Camera camera_setting;
settings::Lidar_matcher setting;

deque<inspec_msg::lidardat> incoming_data;

void Lidar_data_handler(inspec_msg::lidardat msg){
    incoming_data.push_back(msg);
}
void Line_handler(inspec_msg::line2d_array msg){
    if(!incoming_data.empty()){
        double last_dif = msg.header.stamp.toSec() - incoming_data.front().header.stamp.toSec();
    }
    
}


int main(int argc, char **argv){
    ros::init(argc,argv,"Lidar_matcher_node");
    ros::NodeHandle nh = ros::NodeHandle();
    settings::read(camera_setting);
    settings::read(setting);

    ros::Subscriber lidar_data_sub = nh.subscribe("/lidarDat",1,Lidar_data_handler);
    ros::Subscriber line_sub = nh.subscribe("/linedetector/lines2d",1,Line_handler);

    Matched_pub = nh.advertise<inspec_msg::matched_lidar_data_array>("/lidarDat/Matched",1);


    return 0;
}