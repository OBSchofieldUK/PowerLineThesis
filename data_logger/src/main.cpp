#include <ros/ros.h>
#include <ros/package.h>

#include <inspec_msg/line_control_info.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <deque>

using namespace std;

string fileName(){
    time_t now = time(0);
    char* dt = ctime(&now);

    string ret = "dataLog_";
    uint i = 0;
    while(dt[i] !='\0'){
        if(dt[i] != ' '){
            ret+= dt[i];
        }else{
            ret+= '_';
        }
        i++;
    }
    ret+= ".log";
    return ret;
}

std::ofstream logfile;
geometry_msgs::PoseStamped pose;
bool runningSim = false;
void pose_Gazebo_handler(geometry_msgs::PoseStamped msg){
    pose = msg;
    runningSim = true;
    //cout<<msg << endl;
}
void pose_handler(geometry_msgs::PoseStamped msg){
    if(!runningSim){
        pose = msg;
        //cout<<msg << endl;
    }
}
void powerLine_handler(inspec_msg::line_control_info msg){
    logfile << msg.x << "," << msg.y << "," << msg.z << "," << msg.Yaw<< ",";
    if(msg.trusted){
        logfile << 1 << ",";
    }else{
        logfile << 0 << ",";
    }
    logfile << pose.pose.position.x << ",";
    logfile << pose.pose.position.y << ","; 
    logfile << pose.pose.position.z << ",";
    logfile << pose.pose.orientation.w << ",";
    logfile << pose.pose.orientation.x << ",";
    logfile << pose.pose.orientation.y << ","; 
    logfile << pose.pose.orientation.z << ",";  
    logfile << ros::Time::now().toSec();
    logfile << endl << flush;
}


int main(int argc, char* argv[]){
    ros::init(argc,argv,"data_logger");
    ros::NodeHandle nh;
    
    string filePath = ros::package::getPath("data_logger")+ "/Logs/" + fileName();
    logfile.open(filePath);

    ros::Subscriber position_Gazeboo_sub = nh.subscribe("/dataLogger/gazebo_abs_pos_drone", 1, pose_Gazebo_handler);
    ros::Subscriber position_sub = nh.subscribe("/mavros/local_position/pose",1,pose_handler);
    ros::Subscriber powerline_sub = nh.subscribe("/onboard/feedback/powerlinePosition",1,powerLine_handler);

    ros::spin();
    logfile.close();
    return 0;
}

