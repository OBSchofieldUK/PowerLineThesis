#include <inspec_lib/RosConverters.hpp>

namespace{
    void NormalizeLine(math::Vector4d &line2d){
        double l = sqrt(pow(line2d(2),2)+pow(line2d(3),2));
        double sign = 1;
        if(line2d(2)<0) sign = -1;
        line2d(2) = sign*line2d(2)/l;
        line2d(3) = sign*line2d(3)/l;
        if(line2d(2)<0);

        double t = line2d(0)/line2d(2);
        line2d(0) = 0;
        line2d(1) = line2d(1) - t*line2d(3);
    }
}

namespace convert{
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
    rw::math::Vector3D<double> ros2Vector3D(boost::array<double, 3ul> vec){
        return rw::math::Vector3D<double>(vec[0],vec[1],vec[2]);
    }
    boost::array<double, 3ul> Vector3D2ros(rw::math::Vector3D<double> vev){
        return boost::array<double, 3ul>({vev[0], vev[1], vev[2]});
    }
    rw::math::Quaternion<double> ros2Quaternion(boost::array<double, 4ul> vec){
        return rw::math::Quaternion<double>(vec[0],vec[1],vec[2],vec[3]);
    }
    boost::array<double, 4ul> Quaternion2ros(rw::math::Quaternion<double> vec){
        return boost::array<double, 4ul>({vec(0),vec(1),vec(2),vec(3)});
    }
    boost::array<double, 4ul> Quaternion2ros(rw::math::RPY<double> vec){
        return Quaternion2ros(vec.toRotation3D());
    }
    boost::array<double, 4ul> Quaternion2ros(rw::math::Rotation3D<double> vec){
        return Quaternion2ros(rw::math::Quaternion<double>(vec));
    }
    boost::array<double, 4ul> Quaternion2ros(rw::math::EAA<double> vec){
        return Quaternion2ros(vec.toRotation3D());
    }

    rw::math::Vector3D<double> FRU2Image3D(rw::math::Vector3D<double> vec){
        return rw::math::Vector3D<double>(vec[1],vec[2],vec[0]);
    }
    rw::math::Vector3D<double> Image3D2FRU(rw::math::Vector3D<double> vec){
        return rw::math::Vector3D<double>(vec[2],vec[0],vec[1]);
    }
    // ###### FRU Angle Converters ######
    rw::math::Quaternion<double> FRU2Image3D(rw::math::Quaternion<double> vec){
        return rw::math::Quaternion<double>(FRU2Image3D(vec.toRotation3D()));
    }
    rw::math::EAA<double> FRU2Image3D(rw::math::EAA<double> vec){
        return rw::math::EAA<double>(FRU2Image3D(vec.axis()),vec.angle());
    }
    rw::math::RPY<double> FRU2Image3D(rw::math::RPY<double> vec){
        return rw::math::RPY<double>(FRU2Image3D(vec.toRotation3D()));
    }
    rw::math::Rotation3D<double> FRU2Image3D(rw::math::Rotation3D<double> vec){
        return FRU2Image3D(rw::math::EAA<double>(vec)).toRotation3D();
    }

    inspec_msg::line2d_array line2d_array_construct(std::vector<inspec_msg::line2d> lines, uint seq, ros::Time time){
        inspec_msg::line2d_array A;
        A.header.seq = seq;
        A.header.stamp = time;
        A.lines = lines;
        return A;
    }
    inspec_msg::line2d line2ros(const math::Vector4d &line){
        inspec_msg::line2d ret;
        ret.x0 = line(0);
        ret.y0 = line(1);
        ret.dx = line(2);
        ret.dy = line(3);
        return ret;
    }
    inspec_msg::line3d line2ros(const math::Vector7d &line){
        inspec_msg::line3d ret;
        ret.pos = {line(0), line(1), line(2)};
        ret.dir = {line(4), line(5), line(6)};
        return ret;
    }
    math::Vector4d ros2line(const inspec_msg::line2d line){
        math::Vector4d v(line.x0,line.y0,line.dx,line.dy);
        NormalizeLine(v);
        return v;
    }
}

