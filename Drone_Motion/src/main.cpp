

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

deque<geometry_msgs::PoseStamped> DronePosition ;

void NED_QUAT_Position_handler(inspec_msg::position msg){
    msg.position[2] *= -1; //Invert z axis to go upwards instead of down
    Quat angle = convert::ros2Quaternion(msg.Orientation_quat);
    
    RPY a(angle.toRotation3D());
    double h = a[0];
    a[0] = a[2];
    a[2] = h;
    //cout << a << endl;
    angle = Quat(a.toRotation3D());
    TransM wTe(convert::ros2Vector3D(msg.position),angle.toRotation3D());
    TransM sTe = rw::math::inverse(wTs)*wTe;

    // ############# Finalize ###########################
    inspec_msg::position return_msg;
    return_msg.position = convert::Vector3D2ros(sTe.P());
    return_msg.position[1] *= -1;
    return_msg.Orientation_quat = convert::Quaternion2ros(sTe.R());
    return_msg.header = msg.header;

    wTs = wTe;
    motion_pub.publish(return_msg);
}
void onPositionUpdate(geometry_msgs::PoseStamped msg){
    // Pose -> position.x
    DronePosition.push_back(msg);
}
void onImgInput(inspec_msg::head msg){

    if(!DronePosition.empty()){
        double last_dif = msg.stamp.toSec() - DronePosition.front().header.stamp.toSec();

        // ######################## Syncronise Position & Image data #######################
        while(DronePosition.size()>1){
            double dif = msg.stamp.toSec() - DronePosition[1].header.stamp.toSec();
            if(dif<last_dif){
                DronePosition.pop_front();
                last_dif = dif;
            }else{
                break;
            }
        }
        inspec_msg::position msgPos;
        msgPos.header = msg;

        msgPos.position[0] = DronePosition.front().pose.position.x; // North
        msgPos.position[1] = DronePosition.front().pose.position.y; // East
        msgPos.position[2] = -DronePosition.front().pose.position.z; // Down

        msgPos.Orientation_quat[0] = DronePosition.front().pose.orientation.w;
        msgPos.Orientation_quat[1] = DronePosition.front().pose.orientation.x;
        msgPos.Orientation_quat[2] = DronePosition.front().pose.orientation.y;
        msgPos.Orientation_quat[3] = DronePosition.front().pose.orientation.z;
        DronePosition.pop_front();
        
        NED_QUAT_Position_handler(msgPos);
    }else{
        cout << "[drone_motion]: No position data ready: " << msg.seq << endl;
    }
}
int main(int argc, char* argv[]){
    ros::init(argc,argv,"drone_motion");
    ros::NodeHandle nh;

    motion_pub = nh.advertise<inspec_msg::position>("/inspec/daq/DroneInfo/Relative/Position",10);
    global_NED_sub = nh.subscribe("/inspec/daq/DroneInfo/Position",10,NED_QUAT_Position_handler);
    drone_local_sub = nh.subscribe("/mavros/local_position/pose",1, onPositionUpdate);
    camImgSub = nh.subscribe("/inspec/daq/linedetector/gotImage",1,onImgInput);

    ros::spin();
    return 0;
}
