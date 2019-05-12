
#include <ros/ros.h>
#include <ros/package.h>

#include <inspec_msg/position.h>
#include <inspec_lib/converters/RosConverters.hpp>
#include <inspec_lib/Math.hpp>

using namespace std;

ros::Publisher motion_pub;
ros::Subscriber gotImage;


void handler(inspec_msg::head msg){
    inspec_msg::position out;
    out.position = {0,0,0};
    out.Orientation_quat = {0,0,0,0};
    out.header = msg;

    motion_pub.publish(out);
}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"drone_motion");
    ros::NodeHandle nh;

    motion_pub = nh.advertise<inspec_msg::position>("/inspec/daq/DroneInfo/Relative/Position",10);
    gotImage = nh.subscribe("/inspec/daq/linedetector/gotImage",1,handler);

    ros::spin();
    return 0;
}