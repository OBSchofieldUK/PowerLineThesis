

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
    inspec_msg::position return_msg;


    // ############# Relative Movement ##################
    rw::math::Vector3D<double> global_relative_pos = Cur_global_pos - Last_global_pos;
    rw::math::Vector3D<double> local_relative_pos = Last_global_rot*global_relative_pos;
    local_relative_pos[1] *= -1;
    cout << "FRU: " << local_relative_pos[0] << ", " << local_relative_pos[1] << ", " << local_relative_pos[2] << endl;
    return_msg.position = converter::Vector3D2ros(local_relative_pos);

    // ############# Relative Rotation ##################
    rw::math::Rotation3D<double> global_relative_rot = Cur_global_rot * Last_global_rot.inverse(); //TODO What is done in Matlab code
    rw::math::EAA<double> local_relative_rot(global_relative_rot);
    rw::math::Vector3D<double> rotated_dir = Last_global_rot.inverse() * local_relative_rot.axis();
    rotated_dir[2] *= -1;
    local_relative_rot = rw::math::EAA<double>(rotated_dir,local_relative_rot.angle());

    cout << "local_rot: " << rw::math::RPY<>(local_relative_rot.toRotation3D()) << endl;
    return_msg.Orientation_quat = converter::Quaternion2ros(local_relative_rot);
    

    // ############# Finalize ###########################
    Last_global_pos = Cur_global_pos;
    Last_global_rot = Cur_global_rot;

    return_msg.header = msg.header;

    motion_pub.publish(return_msg);
}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"drone_motion");
    ros::NodeHandle nh;

    motion_pub = nh.advertise<inspec_msg::position>("DroneInfo/Relative/Position",10);
    global_NED_sub = nh.subscribe("/DroneInfo/Position",10,NED_QUAT_Position_handler);

    ros::spin();
    return 0;
}