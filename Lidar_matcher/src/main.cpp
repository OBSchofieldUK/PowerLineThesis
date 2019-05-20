


#include <inspec_msg/lidardat.h>
#include <inspec_msg/line2d_array.h>
#include <inspec_msg/line2d.h>
#include <inspec_msg/matched_lidar_data_array.h>
#include <inspec_msg/matched_lidar_data.h>

#include <inspec_lib/settings/SettingStructs.hpp>
#include <inspec_lib/settings/ReadSettings.hpp>
#include <inspec_lib/converters/OtherConverters.hpp>
#include <inspec_lib/converters/CoordinateConverters.hpp>
#include <inspec_lib/converters/RosConverters.hpp>
#include <inspec_lib/Math.hpp>

#include <rw/math.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <deque>
#include <vector>

using namespace std;

ros::Publisher Matched_pub;
settings::Camera camera_setting;
settings::Lidar lidar_setting;

rw::math::Transform3D<double> camTlidar;

deque<inspec_msg::lidardat> incoming_data;

cv::Point2f point3dToImg(rw::math::Vector3D<double> point){
    cv::Point2f result;
    result.x = point[0]*(camera_setting.d/point[2]);
    result.y = point[1]*(camera_setting.d/point[2]);

    result.y *= camera_setting.pixel_height/camera_setting.Chip_size_y;
    result.x *= camera_setting.pixel_width/camera_setting.Chip_size_x;

    return result;
}
rw::math::Vector3D<double> get3dPoint(const double &distance,const vector<double> &RPY){
    rw::math::Transform3D<double> lidarTPoint = convert::ToTransform({0,0,0},RPY); //RotateAround X axis
    return rw::math::inverse(camTlidar)*rw::math::inverse(lidarTPoint)*rw::math::Vector3D<double>(0,0,distance);
}
cv::Point2f getPoint(const double &distance,const vector<double> RPY){
    return point3dToImg(get3dPoint(distance,RPY));
}
void getCorners(vector <cv::Point2f> &dst, uint seg_num,double &distance){
    dst.push_back(getPoint(distance,{lidar_setting.seg_angle[seg_num]+lidar_setting.segment_H_angle/2,
                                     lidar_setting.segment_V_angle/2,0}));

    dst.push_back(getPoint(distance,{lidar_setting.seg_angle[seg_num]+lidar_setting.segment_H_angle/2,
                                     -lidar_setting.segment_V_angle/2,0}));

    dst.push_back(getPoint(distance,{lidar_setting.seg_angle[seg_num]-lidar_setting.segment_H_angle/2,
                                     lidar_setting.segment_V_angle/2,0}));

    dst.push_back(getPoint(distance,{lidar_setting.seg_angle[seg_num]-lidar_setting.segment_H_angle/2,
                                     -lidar_setting.segment_V_angle/2,0}));
    //dst.push_back(getPoint(distance,{math::deg2rad(lidar_setting.seg_angle[seg_num]),0,0}));
}
bool between(cv::Point2f point, cv::Point2f lim1, cv::Point2f lim2){
    if(lim1.x > lim2.x){
        cv::Point2f h = lim1;
        lim1 = lim2;
        lim2 = h;
    }
    return lim1.x < point.x && point.x < lim2.x; 
}

void Lidar_data_handler(inspec_msg::lidardat msg){
    //cout << "Lidar Data recived" <<endl;
    incoming_data.push_back(msg);
}
void Line_handler(inspec_msg::line2d_array msg){
    //cout << "Line data recived" << endl;
    cv::Mat img(camera_setting.pixel_height, camera_setting.pixel_width, CV_8UC3, cv::Scalar(0,0,0));
    if(!incoming_data.empty()){
        double last_dif = msg.header.stamp.toSec() - incoming_data.front().header.stamp.toSec();
        
        // ######################## Syncronise Lidar & Image data #######################
        for(uint i = 1; i < incoming_data.size();i++){
            double dif = msg.header.stamp.toSec() - incoming_data[i].header.stamp.toSec();
            if(last_dif-dif>-0.001){
                incoming_data.pop_front();
                i--;
                last_dif = dif;
            }else{
                break;
            }
        }
        //DEBUG
        for(uint i = 0; i < msg.lines.size(); i++){
            math::mathLine2d l = convert::ros2mathLine(msg.lines[i]);
            math::drawMathLine(img,l,cv::Scalar(255,255,255));
        }

        inspec_msg::matched_lidar_data_array match_array;
        inspec_msg::lidardat lidar = incoming_data.front();
        incoming_data.pop_front();

        // ##################### Match Lidar and Image data ########################
        if(lidar.distance.size() ==  lidar_setting.number_of_segments){
            for(uint i = 0; i < lidar.distance.size();i++){
                if(lidar.distance[i] > 0.0001){
                    vector<cv::Point2f> points;

                    // Find Lidar segmants
                    getCorners(points,i,lidar.distance[i]);
                    math::mathLine2d l1 = math::constructMathLine(points[0],points[2]);
                    math::mathLine2d l2 = math::constructMathLine(points[1],points[3]);

                    //Find Line going through segment
                    for(auto &line: msg.lines){
                        math::mathLine2d l = convert::ros2mathLine(line);
                        cv::Point2f p1 = math::intersection(l,l1);
                        cv::Point2f p2 = math::intersection(l,l2);
                        if(between(p1,points[0],points[2]) || between(p2,points[1],points[3])){
                            //cout << "Match: " << i << ", " << line.id << std::endl;
                            inspec_msg::matched_lidar_data match;

                            //Simple aproximation of distance - Should be using a beam projection using the pixel point and the lidar distance plane
                            match.distance = lidar.distance[i]*cos(math::deg2rad(lidar_setting.seg_angle[i]));
                            match.matched_line = line.id;
                            if(between(p1,points[0],points[2])){
                                match.x = p1.x;
                                match.y = p1.y;
                            }else{
                                match.x = p2.x;
                                match.y = p2.y;
                            }
                            match_array.data.push_back(match);
                            break; // Find Only one line per lidar segment.
                        }
                    }
                    //DEBUG
                    //cout << "Printing Points: " << points.size() << endl;
                    for(auto &p: points){
                        cv::circle(img,convert::real2imgCoord(p,cv::Size(camera_setting.pixel_width,camera_setting.pixel_height)),3,cv::Scalar(0,255,0),2);
                    }
                }
            }
        }else{
            throw "Wrong amount of Lidar segments";
        }

        if(!match_array.data.empty()){
            match_array.header = msg.header;
            Matched_pub.publish(match_array);
        }
    }
    
    cv::imshow("test",img);
    cv::waitKey(1);
}
void ShowImage(const string &name, const cv::Mat &img, int x = 50, int y = 50 ){
    cv::namedWindow(name,cv::WINDOW_NORMAL);
    cv::moveWindow(name,x,y);
    cv::resizeWindow(name, 800,500);
    cv::imshow(name, img);
}
int main(int argc, char **argv){
    ros::init(argc,argv,"Lidar_matcher_node");
    ros::NodeHandle nh = ros::NodeHandle();
    settings::read(camera_setting);
    settings::read(lidar_setting);
    cout << camera_setting << endl;

    camTlidar = rw::math::inverse(convert::ToTransform(lidar_setting.XYZ_lidarTcam, lidar_setting.RPY_lidarTcam));
    cout << "####### Lidar Position #######" << endl;
    cout << camTlidar.P() << endl;
    rw::math::RPY<> a(camTlidar.R());
    cout << "RPY: " << math::rad2deg(a[0]) << ", " << math::rad2deg(a[1]) << ", " << math::rad2deg(a[2]) << endl;  



    ros::Subscriber lidar_data_sub = nh.subscribe("/inspec/daq/lidarDat",1,Lidar_data_handler);
    ros::Subscriber line_sub = nh.subscribe("/inspec/daq/linedetector/lines2d",1,Line_handler);

    Matched_pub = nh.advertise<inspec_msg::matched_lidar_data_array>("/inspec/daq/lidarDat/Matched",1);

    cv::Mat BLACK(camera_setting.pixel_height, camera_setting.pixel_width, CV_8UC3, cv::Scalar(0,0,0));
    ShowImage("test",BLACK);
    cv::waitKey(1);
    ros::spin();


    return 0;
}