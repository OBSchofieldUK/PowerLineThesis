#ifndef SETTING_STRUCTS_HPP_
#define SETTING_STRUCTS_HPP_

#include <string>
#include <vector>
#include <iostream>

typedef unsigned char uchar;

namespace settings{ 
    struct AirSimRecording{
        std::string Recording_folder;
        double SlowDownFactor;
        uint start_at_img_num;
        uint end_at_img_num;
    };
    const AirSimRecording AirSimRecordingDefault = {" ",1.0,0,6000};
    
    struct PLineD{
        uint canny_filter_size;
        uchar canny_treshold_low;
        uchar canny_treshold_high;

        uint segcut_min_length;
        double segcut_max_angle;
        uint segcut_step_size;

        uint covariance_ratio;

        uint group_min_end_length;
        uint group_min_start_length;
        double group_max_angle_dif;
        uint group_max_line_dist;

        bool Parrallel_active;
        double Parralel_max_angle;

        bool debug;
    };
    const PLineD PLineDDefault = {3,36,150,15,30.0,5,600,400,100,5.0,10,false,5.0,false};

    struct Image_processing_node{
        bool debug;
        bool show_incomming_image;
        bool show_final_image;
        bool press_to_continue;
        std::string Image_topic;
    };
    const Image_processing_node Image_processing_node_Default = {false,true,true,false,"/webcam/image_raw"};

    struct Vannishing_Point_Filter{
        bool debug;
        uint max_cluster_error;
        uint min_cluster_error;
    };
    const Vannishing_Point_Filter Vannishing_Point_Filter_Default = {false, 300, 50};

    struct Proximity_Filter{
        bool debug;
    };
    const Proximity_Filter Proximity_Filter_Default = {false};

    struct Camera{
        uint pixel_width;
        uint pixel_height;
        double Chip_size_mm;
        double Chip_size_x;
        double Chip_size_y;
        double focal_length_mm;
        double d;
        double FOV;
        double RPY_camTdrone[3];
        double XYZ_camTdrone[3];
    };
    const Camera Camera_Default = {1280,720,0,0,0,0,0,60,{0,0,0},{0,0,0}};
    std::ostream& operator<<(std::ostream& os, const Camera& cam);

    struct Kalman_LineXmove{
        bool debug;
    };
    const Kalman_LineXmove Kalman_LineXmove_Default = {false};

    struct Lidar{
        double RPY_lidarTcam[3];
        double XYZ_lidarTcam[3];
        uint number_of_segments;
        double segment_H_angle;
        double segment_V_angle;
        std::vector<double> seg_angle;

    };
    const Lidar Lidar_matcher_Default = {{0,0,0},{0,0,5},8,48.0/8,0.3};

}


#endif
