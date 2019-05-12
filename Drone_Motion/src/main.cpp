

#include <rw/math.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

#include <inspec_msg/position.h>
#include <inspec_lib/converters/RosConverters.hpp>
#include <inspec_lib/Math.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <deque>


using namespace std;

ros::Publisher motion_pub;
ros::Subscriber global_NED_sub;
ros::Subscriber drone_local_sub;
ros::Subscriber camImgSub;

typedef rw::math::Vector3D<double> Vect;
typedef rw::math::Quaternion<double> Quat;
typedef rw::math::Rotation3D<double> RotM;
typedef rw::math::Transform3D<double> TransM;
typedef rw::math::RPY<double> RPY;

TransM wTs;

deque<geometry_msgs::PoseStamped> imgMsg ;

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
void onPositionUpdate(geometry_msgs::PoseStamped msg){
    // Pose -> position.x
    imgMsg.push_back(msg);

}
void onImgInput(inspec_msg::head inImg){

    double last_dif = inImg.stamp.toSec() - imgMsg.front().header.stamp.toSec();

    // ######################## Syncronise Lidar & Image data #######################
    for(uint i = 1; i < imgMsg.size();i++){
        double dif = inImg.stamp.toSec() - imgMsg[i].header.stamp.toSec();
        if(dif<last_dif){
            imgMsg.pop_front();
            i--;
            last_dif = dif;
        }else{
            break;
        }
    }

    inspec_msg::position msgPos;
    msgPos.header = inImg;

    msgPos.position[0] = imgMsg.front().pose.position.x;
    msgPos.position[1] = imgMsg.front().pose.position.y;
    msgPos.position[2] = imgMsg.front().pose.position.z;

    msgPos.Orientation_quat[0] = imgMsg.front().pose.orientation.w;
    msgPos.Orientation_quat[1] = imgMsg.front().pose.orientation.x;
    msgPos.Orientation_quat[2] = imgMsg.front().pose.orientation.y;
    msgPos.Orientation_quat[3] = imgMsg.front().pose.orientation.z;

    NED_QUAT_Position_handler(msgPos);
}
int main(int argc, char* argv[]){
    ros::init(argc,argv,"drone_motion");
    ros::NodeHandle nh;

    motion_pub = nh.advertise<inspec_msg::position>("/inspec/daq/DroneInfo/Relative/Position",10);
    global_NED_sub = nh.subscribe("/inspec/daq/DroneInfo/Position",10,NED_QUAT_Position_handler);
    drone_local_sub = nh.subscribe("/mavros/local_position/pose",1, onPositionUpdate);
    camImgSub = nh.subscribe("/inspec/daq/linedetection/gotImage",1,onImgInput);


    ros::spin();
    return 0;
}
