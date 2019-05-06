
#include <ros/ros.h>

#include <inspec_msg/lidardat.h>
#include <inspec_msg/line2d_array.h>
#include <inspec_msg/line2d.h>
#include <inspec_msg/matched_lidar_data_array.h>
#include <inspec_msg/matched_lidar_data.h>

#include <inspec_lib/settings/SettingStructs.hpp>
#include <inspec_lib/settings/ReadSettings.hpp>

using namespace std;

ros::Publisher Matched_pub;
settings::Camera camera_setting;

void Lidar_data_handler(inspec_msg::lidardat msg){

}
void Line_handler(inspec_msg::line2d_array msg){

}


int main(int argc, char **argv){
    ros::init(argc,argv,"Lidar_matcher_node");
    ros::NodeHandle nh = ros::NodeHandle();
    settings::read(camera_setting);

    ros::Subscriber lidar_data_sub = nh.subscribe("/lidarDat",1,Lidar_data_handler);
    ros::Subscriber line_sub = nh.subscribe("/linedetector/lines2d",1,Line_handler);

    Matched_pub = nh.advertise<inspec_msg::matched_lidar_data_array>("/lidarDat/Matched",1);


    return 0;
}