

#include <rw/math.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

#include <inspec_msg/position.h>
#include <inspec_lib/converters/RosConverters.hpp>
#include <inspec_lib/Math.hpp>

using namespace std;

ros::Publisher motion_pub;
ros::Subscriber global_NED_sub;
ros::Subscriber drone_local_sub;


typedef rw::math::Vector3D<double> Vect;
typedef rw::math::Quaternion<double> Quat;
typedef rw::math::Rotation3D<double> RotM;
typedef rw::math::Transform3D<double> TransM;
typedef rw::math::RPY<double> RPY;

TransM wTs;
void NED_QUAT_Position_handler(inspec_msg::position msg){
    msg.position[2] *= -1; //Invert z axis to go upwards instead of down
    TransM wTe(convert::ros2Vector3D(msg.position),convert::ros2Quaternion(msg.Orientation_quat).toRotation3D());
    TransM sTe = rw::math::inverse(wTs)*wTe; 

    // ############# Finalize ###########################
    inspec_msg::position return_msg;
    return_msg.position = convert::Vector3D2ros(sTe.P());
    return_msg.Orientation_quat = convert::Quaternion2ros(sTe.R());
    return_msg.header = msg.header;

    wTs = wTe;
    motion_pub.publish(return_msg);
}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"drone_motion");
    ros::NodeHandle nh;

    motion_pub = nh.advertise<inspec_msg::position>("DroneInfo/Relative/Position",10);
    global_NED_sub = nh.subscribe("/DroneInfo/Position",10,NED_QUAT_Position_handler);
    //drone_local_sub = nh.subscribe("/mavros/local_position/pose",1,);

    ros::spin();
    return 0;
}