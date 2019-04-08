

#include <rw/math.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

#include <inspec_msg/position.h>
#include <inspec_lib/RosConverters.hpp>
#include <inspec_lib/Math.hpp>

using namespace std;

ros::Publisher motion_pub;
ros::Subscriber global_NED_sub;


rw::math::Vector3D<double> Last_global_pos;
rw::math::Rotation3D<double> Last_global_rot;
void NED_QUAT_Position_handler(inspec_msg::position msg){
    rw::math::Vector3D<double> Cur_global_pos = converter::ros2Vector3D(msg.position);
    rw::math::Rotation3D<double> Cur_global_rot = converter::ros2Quaternion(msg.Orientation_quat).toRotation3D();

    rw::math::Vector3D<double> global_relative_pos = Cur_global_pos - Last_global_pos;
    cout << "Global relative pos: " << global_relative_pos << endl;
    rw::math::Vector3D<double> local_relative_pos = Last_global_rot*global_relative_pos;
    cout << "Local relative pos: " << local_relative_pos << endl; 


    Last_global_pos = Cur_global_pos;
    Last_global_rot = Cur_global_rot;
}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"drone_motion");
    ros::NodeHandle nh;

    motion_pub = nh.advertise<inspec_msg::position>("DroneInfo/Relative/Position",10);
    global_NED_sub = nh.subscribe("/DroneInfo/Position",10,NED_QUAT_Position_handler);

    ros::spin();
    return 0;
}