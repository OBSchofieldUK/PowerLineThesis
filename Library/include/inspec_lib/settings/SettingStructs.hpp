#ifndef SETTING_STRUCTS_HPP_
#define SETTING_STRUCTS_HPP_

#include <string>
#include <iostream>

typedef unsigned char uchar;

namespace settings{
    
    struct AirSimRecording{
        std::string Recording_folder;
        double SlowDownFactor;
    };
    const AirSimRecording AirSimRecordingDefault = {" ",1.0};
    
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
    const PLineD PLineDDefault = {3,36,150,15,30.0,5,600,100,400,5.0,10,false,5.0,false};

    struct Image_processing_node{
        bool debug;
        bool show_incomming_image;
        bool show_final_image;

        bool press_to_continue;
    };
    const Image_processing_node Image_processing_node_Default = {false,true,true,false};
}


#endif
