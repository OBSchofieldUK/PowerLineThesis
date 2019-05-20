#include <ros/ros.h>

#include <inspec_lib/settings/ReadSettings.hpp>
#include <inspec_lib/settings/SettingStructs.hpp>
#include <inspec_lib/converters/CoordinateConverters.hpp>
#include <inspec_lib/converters/RosConverters.hpp>
#include <inspec_lib/converters/OtherConverters.hpp>
#include <inspec_lib/Math.hpp>

#include <inspec_msg/line3d_array.h>
#include <inspec_msg/line_control_info.h>

#include <rw/math.hpp>

#include <iostream>

using namespace std;

ros::Publisher powerline_position_pub;

settings::Camera cam_setting;
rw::math::Transform3D<double> camTdrone;

rw::math::Vector3D<double> closestPoint(const inspec_msg::line3d line){
    rw::math::Vector3D<double> pos(line.pos[0],line.pos[1],line.pos[2]);
    rw::math::Vector3D<double> dir(line.dir[0],line.dir[1],line.dir[2]);
    rw::math::Vector3D<double> point(0,0,0);
    double t = (-2*dot(dir,pos-point))/(2* dot(dir,dir));
    return pos+t*dir;
}
bool isTrusted(const inspec_msg::line3d &line){
    bool trust = false;
    if(line.Lidar_fix){
        trust = true;
    }else if(line.B_error > 20){
        trust = true;
    }
    if(line.pos[2] < 5 && !line.Lidar_fix){
        trust = false;
    }
    return trust;
}

//This is a dummy selector just for show casing that one part of the program should select
//the powerline that the drone is gonna follow
inspec_msg::line3d selector(const inspec_msg::line3d_array &line_array){
    for(auto l: line_array.lines){
        if(isTrusted(l)) return l;
    } 
    return line_array.lines[0];
}

inspec_msg::line_control_info controlInfo(const inspec_msg::line3d &line){
    rw::math::Vector3D<double> pos = closestPoint(line);
    /*pos = convert::Image3D2FRU(pos);
    pos = camTdrone * pos;*/

    rw::math::Vector3D<double> dir(line.dir[0],line.dir[1],line.dir[2]);
    //dir = convert::Image3D2FRU(dir);
    //dir = camTdrone.R()*dir;
    
    inspec_msg::line_control_info info;
    info.x = pos[0];
    info.y = pos[1];
    info.z = pos[2];
    info.Yaw = atan(dir[1]/dir[0]);

    info.trusted = isTrusted(line);
    
    return info;
}

void powerline_handler(inspec_msg::line3d_array msg){
    if(!msg.lines.empty()){
        auto line = selector(msg);
        auto info = controlInfo(line);
        powerline_position_pub.publish(info);
    }
}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"powerline_selector");
    ros::NodeHandle nh;

    settings::read(cam_setting);
    camTdrone = convert::ToTransform(cam_setting.XYZ_camTdrone,cam_setting.RPY_camTdrone);

    ros::Subscriber powerline_sub = nh.subscribe("/inspec/daq/Estimator/lines3d",1,powerline_handler);
    powerline_position_pub = nh.advertise<inspec_msg::line_control_info>("/onboard/setpoint/inspect",1);

    ros::spin();
    return 0;
}

