#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

#include <chrono>
#include <thread>

#include <rw/math.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <inspec_msg/position.h>

#include <inspec_lib/settings/ReadSettings.hpp>

using namespace std;

typedef Eigen::Matrix<double,6,1> Vector6d;

settings::AirSimRecording setting;

struct airSimData{
    rw::math::Vector3D<double> pos;
    rw::math::Quaternion<double> rot;
    double t;
    string imgPath;
    string depthPath;
};

// ########################## Other #################################
void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

void getAirSimData(const string &line, airSimData &dst){
    vector<string> rows;
    split(line, '\t',rows);

    if(rows.size() < 8){
        throw "To Little Data";
    }
    dst.t = std::stod(rows[0]);
    for(int i = 1; i < 4; i++)
        dst.pos[i-1] = std::stod(rows[i]);
    
    for(int i = 4; i < 8; i++)
        dst.rot(i-4) = std::stod(rows[i]);

    vector<string> imgPaths;
    split(rows[8],';',imgPaths);
    dst.imgPath = string(setting.Recording_folder) + "images/" +imgPaths[0];
    dst.depthPath = string(setting.Recording_folder) + "images/" +imgPaths[1];
    
}

void ImageWindow(const string &name, int x = 50, int y = 50 ){
    cv::Mat img(600, 500, CV_8UC3, cv::Scalar(0,0,0));
    cv::namedWindow(name,cv::WINDOW_NORMAL);
    cv::moveWindow(name,x,y);
    cv::resizeWindow(name, 600,500);
    cv::imshow(name, img);
}

std_msgs::Header header;
ros::Publisher img_pub;
void publishImageToRos(cv::Mat img){
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    img_pub.publish(img_msg);
}

ros::Publisher position_pub;
void publishPositionToRos(airSimData data){
    inspec_msg::position msg;
    msg.header.seq = header.seq++;
    msg.header.stamp = header.stamp;
    msg.position = {data.pos[0], data.pos[1], data.pos[2]};
    msg.Orientation_quat = {data.rot(0),data.rot(1),data.rot(2),data.rot(3)};
    position_pub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"Airsim_images");
    ros::NodeHandle nh = ros::NodeHandle();
    img_pub = nh.advertise<sensor_msgs::Image>("/webcam/image_raw",10);
    position_pub = nh.advertise<inspec_msg::position>("/inspec/daq/DroneInfo/Position",10);
    settings::read(setting);
    
    // ############# Open Recording File #################
    ifstream file;
    file.open(setting.Recording_folder+"airsim_rec.txt");
    if (!file){
        cout << "Unable to open file at: " << setting.Recording_folder+"airsim_rec.txt"  << endl;
        cout << "Please change the Path in settings.jason under AirSimRecording: Record_folder" << endl;
        exit(1);
    }
    bool first = true;

    //Create Image Windows
    //ImageWindow("Original",50,50);
    //ImageWindow("DepthMap",650,50);
    
    // Loop thrugh 
    string line;
    getline(file,line);
    for(uint i = 0; i < setting.start_at_img_num; i++){
         getline(file,line);
         header.seq++;
    }
    
    while (getline(file,line) && ros::ok() && header.seq < setting.end_at_img_num+1){
        airSimData data;
        getAirSimData(line,data);
        cv::Mat img = cv::imread(data.imgPath);
        cv::Mat depth = cv::imread(data.depthPath);
        if (img.empty() || depth.empty()) {
            std::cout << "Input image not found at '" << data.imgPath << "' or '" << data.depthPath << "'\n";
            return 1;
        }
        publishImageToRos(img);
        publishPositionToRos(data);
        ros::spinOnce();
        if(first){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            first = false;
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(long(1000*setting.SlowDownFactor/30)));
        }
        
        
    }
    return 0;
}