#include <ros/ros.h>


int main(int argc, char* argv[]){
    ros::init(argc,argv,"powerline selector");
    ros::NodeHandle nh;

    //motion_pub = nh.advertise<inspec_msg::position>("DroneInfo/Relative/Position",10);
    //global_NED_sub = nh.subscribe("/DroneInfo/Position",10,NED_QUAT_Position_handler);

    ros::spin();
    return 0;
}