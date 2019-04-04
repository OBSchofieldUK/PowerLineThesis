#include "../include/inspec_lib/RosConverters.hpp"


namespace converter{
    math::mathLine2d ros2mathLine(inspec_msg::line2d line){
        math::mathLine2d ret;
        double t = line.x0/line.dx;
        ret.b =  line.y0-line.dy*t;
        ret.a = line.dy/line.dx;
        
        return ret;
    }
    inspec_msg::line2d mathLine2ros(math::mathLine2d mline, uint id){
        inspec_msg::line2d msg_line;
        msg_line.dx = 1;
        msg_line.dy = mline.a;
        msg_line.x0 = 0;
        msg_line.y0 = mline.b;
        msg_line.id = id;
        return msg_line;
    }
}