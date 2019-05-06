#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <deque>
#include <map>

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <rw/math.hpp>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <inspec_msg/line2d.h>
#include <inspec_msg/line2d_array.h>
#include <inspec_msg/line3d.h>
#include <inspec_msg/line3d_array.h>
#include <inspec_msg/position.h>

#include <inspec_lib/Math.hpp>
#include <inspec_lib/converters/RosConverters.hpp>
#include <inspec_lib/converters/CoordinateConverters.hpp>

#include <inspec_lib/settings/ReadSettings.hpp>
#include <inspec_lib/settings/SettingStructs.hpp>

#define MAX_UNOBSERVED_STATES_BEFORE_DELETION 4ul
#define MIN_OBSERVATION_TOO_BEE_TRUSTED 4ul
#define MAX_UNOBSERVED_STATES_BEFORE_DELETION_UNTRUSTED_LINE 1ul
#define MAX_ERROR_FOR_EST_MATCHING 1.5f

#define X0 0
#define Y0 1
#define Z0 2
#define dX 4
#define dY 5
#define dZ 6

using Eigen::MatrixXd;
using namespace std;

typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,4,1> Vector4d;
typedef Eigen::Matrix<double,7,7> Matrix7;
typedef Eigen::Matrix<double,4,4> Matrix4;
typedef Eigen::Matrix<double,4,7> Matrix4x7;
typedef Eigen::Matrix<double,7,4> Matrix7x4;

struct point{
    double x;
    double y;
};
struct line{
    double a;
    double b;
};
struct camera{
    double d;       //in Meters
    point pixel;    //Resolution
    point size;     //Physical chipsize in meters
};
struct lineEstimate{
    Vector7d X_hat;
    Matrix7 P;
    Vector4d line2d;
    inspec_msg::line2d last_correct;


    bool trust_estimate;
    long id;
    size_t lastObserved;
    size_t consecutiveObservations;
    size_t observations;
};

settings::Camera setting_camera;

// #### ROS Variables #####
ros::Subscriber line_sub;
ros::Subscriber position_sub;
ros::Publisher  line_pub;
ros::Publisher  line2Dest_pub;


// #### Estimation Variables ####
Vector7d Sigma_u; 
Matrix7  Sigma_u_diag;
Vector4d Sigma_r;
Matrix4  Sigma_r_diag;

Matrix7 G;
Matrix7 Q;

Vector7d P_initial;
Matrix7 P_initial_diag;

Vector7d X_hat_initial_guess;

map<long,lineEstimate> ActiveLines;

// #### Data Aquisition variables ####
size_t image_seq;
size_t position_seq;
long max_line_id;
rw::math::Vector3D<double> last_pos;
rw::math::Quaternion<double> last_ori;
deque<inspec_msg::position> positionQueue(10);

// #### System Constants ####
camera currentCam;

// ######################### MATLAB Functions ####################################
Matrix4x7 H_matrix(Vector7d vec, camera cam){
    double X_scale = cam.pixel.x/cam.size.x;
    double Y_scale = cam.pixel.y/cam.size.y;
    double &d = cam.d;
    double &x0 = vec(0);
    double &y0 = vec(1);
    double &z0 = vec(2);
    double &x =  vec(4);
    double &y =  vec(5);
    double &z =  vec(6);
    Eigen::Matrix<double,4,7> ret;
    ret.row(0) <<               (X_scale*d)/z0,                            0,                                                                                                       -(X_scale*d*x0)/(z0*z0), 0,                   0,                    0,                                                                                 0;
    ret.row(1) <<                            0,               (Y_scale*d)/z0,                                                                                                       -(Y_scale*d*y0)/(z0*z0), 0,                    0,                    0,                                                                                 0;
    ret.row(2) << -(X_scale*d*z)/(z0*(z + z0)),                            0, (X_scale*d*x)/(z0*(z + z0)) - (X_scale*d*(x*z0 - x0*z))/(z0*(z + z0)*(z + z0)) - (X_scale*d*(x*z0 - x0*z))/((z0*z0)*(z + z0)), 0, (X_scale*d)/(z + z0),                    0, - (X_scale*d*x0)/(z0*(z + z0)) - (X_scale*d*(x*z0 - x0*z))/(z0*(z + z0)*(z + z0));
    ret.row(3) <<                            0, -(Y_scale*d*z)/(z0*(z + z0)), (Y_scale*d*y)/(z0*(z + z0)) - (Y_scale*d*(y*z0 - y0*z))/(z0*(z + z0)*(z + z0)) - (Y_scale*d*(y*z0 - y0*z))/((z0*z0)*(z + z0)), 0,                    0, (Y_scale*d)/(z + z0), - (Y_scale*d*y0)/(z0*(z + z0)) - (Y_scale*d*(y*z0 - y0*z))/(z0*(z + z0)*(z + z0));
    return ret;
}
Matrix7 F_matrix(rw::math::Vector3D<double> vec, rw::math::Quaternion<double> angle){
    rw::math::Transform3D<double> trans = rw::math::inverse(rw::math::Transform3D<double>(vec,angle.toRotation3D()));
    Eigen::Matrix<double,7,7> ret;
    for( int x = 0; x < ret.rows(); x++){      
        for( int y = 0; y < ret.cols();y++){
            if( x < 3 && y < 3){
                ret(x,y) = trans(x,y);
                ret(x+4,y+4) =  trans(x,y); 
            }else if(y == 3 && x < 3){
                ret(x,y) = vec(x);
            }else if(!(y>3 && x>3)){
                ret(x,y) = 0;
            } 
        }
    }
    ret(3,3) = 1;
    return ret;
}
Matrix7 F_matrix(Vector7d vec){
    rw::math::Vector3D<double> v(vec(0),vec(1),vec(2));
    rw::math::Quaternion<double> q(rw::math::EAA<double>(vec(4),vec(5),vec(6)).toRotation3D());
    return F_matrix(v,q);
}
Vector7d equationSolve(rw::math::Transform3D<double> move, line cur, line last,camera cam){
    double &d = cam.d;
    double &a1 = cur.a;
    double &b1 = cur.b;
    double &a2 = last.a;
    double &b2 = last.b;

    double &t00 = move(0,0);
    double &t10 = move(1,0);
    double &t20 = move(2,0);

    double &t01 = move(0,1);
    double &t11 = move(1,1);
    double &t21 = move(2,1);

    double &t02 = move(0,2);
    double &t12 = move(1,2);
    double &t22 = move(2,2);

    double &t03 = move(0,3);
    double &t13 = move(1,3);
    double &t23 = move(2,3);

    double a12 = pow(a1,2);
    double a22 = pow(a2,2);
    double b12 = pow(b1,2);
    double b22 = pow(b2,2);
    double d4 = pow(d,4);
    double d3 = pow(d,3);
    double d2 = pow(d,2);
    double common = 1/(  a12*a22*d4*            (pow(t01,2) + pow(t02,2)) 
                        + 2*a12*a2*b2*d3*       (t01*t21 + t02*t22) 
                        - 2*a12*a2*d4*          (t01*t11 + t02*t12) 
                        + a12*b22*d2*           (pow(t21,2) + pow(t22,2)) 
                        - 2*a12*b2*d3*          (t11*t21 + t12*t22) 
                        + a12*d4*               (pow(t11,2) + pow(t12,2)) 
                        - 2*a1*a22*d3*t00*      (b1*t02 - d*t01) 
                        - 2*a1*a2*b1*b2*d2*     (t00*t22 + t02*t20) 
                        + 2*a1*a2*b1*d3*        (t00*t12 + t02*t10) 
                        + 2*a1*a2*b2*d3*        (t00*t21 + t01*t20) 
                        - 2*a1*a2*d4*           (t00*t11 + t01*t10) 
                        - 2*a1*b1*d*b2*         (b2*t20*t22 - d*t10*t22) 
                        + 2*a1*b1*d2*           (b2*t12*t20 - d*t10*t12) 
                        + 2*a1*b2*d2*           (b2*t20*t21 - d*t10*t21) 
                        - 2*a1*                 (b2*d3*t11*t20 - d4*t10*t11) 
                        + a22*b12*d2*           (pow(t00,2) + pow(t01,2)) 
                        + 2*a22*b1*d3*t01*t02 
                        + a22*d4*               (pow(t00,2) + pow(t02,2)) 
                        + 2*a2*b12*b2*d*        (t00*t20 + t01*t21) 
                        - 2*a2*b12*d2*          (t00*t10 + t01*t11) 
                        + 2*a2*b1*b2*d2*        (t01*t22 + t02*t21) 
                        - 2*a2*b1*d3*           (t01*t12 + t02*t11) 
                        + 2*a2*b2*d3*           (t00*t20 + t02*t22) 
                        - 2*a2*d4*              (t00*t10 + t02*t12) 
                        + b12*b22*              (pow(t20,2) + pow(t21,2)) 
                        - 2*b12*b2*d*           (t10*t20 + t11*t21) 
                        + b12*d2*               (pow(t10,2) + pow(t11,2)) 
                        + 2*b1*b22*d*           (t21*t22 - t11*t22) 
                        - 2*b1*b2*d2*t12*t21 
                        + 2*b1*d3*t11*t12 
                        + b22*d2*               (pow(t20,2) + pow(t22,2)) 
                        - 2*b2*d3*              (t10*t20 + t12*t22) 
                        + d4*                   (pow(t10,2) + pow(t12,2))
    );
    common = pow(common,1/2);
    
    Vector7d ret;
    ret(0) = 0;
    ret(1) = -(b1*(b2*t23 - d*t13 + a2*d*t03))/(b1*b2*t21 - d*d*t12 - b1*d*t11 + b2*d*t22 + a2*d*d*t02 + a2*b1*d*t01);
    ret(2) = -(b2*d*t23 - d*d*t13 + a2*d*d*t03)/(b1*b2*t21 - d*d*t12 - b1*d*t11 + b2*d*t22 + a2*d*d*t02 + a2*b1*d*t01);
    ret(4) = -common*(b1*b2*t21 - d2*t12 - b1*d*t11 + b2*d*t22 + a2*d2*t02 + a2*b1*d*t01);
    ret(5) = common*(b1*b2*t20 - b1*d*t10 + a1*d2*t12 - a1*a2*d2*t02 + a2*b1*d*t00 - a1*b2*d*t22);
    ret(6) = d*common*(b2*t20 - d*t10 + a1*b2*t21 + a2*d*t00 - a1*d*t11 + a1*a2*d*t01);
    return ret;
}
void NormalizeLine(Vector7d &line3d){
    double l = sqrt(pow(line3d(dX),2)+pow(line3d(dY),2)+pow(line3d(dZ),2));
    double sign = 1;
    if(line3d(dX)<0) sign = -1;
    line3d(dX) =sign*line3d(dX)/l;
    line3d(dY) =sign*line3d(dY)/l;
    line3d(dZ) =sign*line3d(dZ)/l;

    line3d(3) = 1;
    double t = line3d(X0)/line3d(dX);
    line3d(X0) = 0;
    line3d(Y0) -= t*line3d(dY);
    line3d(Z0) -= t*line3d(dZ);
}
void NormalizeLine(Vector4d &line2d){
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
void realToImageLine(Vector4d &Line2D,camera cam){
    double Xs = cam.pixel.x/cam.size.x;
    double Ys = cam.pixel.y/cam.size.y;
    for(int i = 0; i < 2; i++){
        Line2D(i) = Line2D(i) * Xs;
        Line2D(i+2) = Line2D(i+2) * Ys;
    }
    NormalizeLine(Line2D);
}
Vector4d line3dTo2d(Vector7d line3d, camera cam){
    Vector4d Line2D;
    for(int i = 0; i < 2; i++){
        Line2D(i) = line3d(i)*cam.d/line3d(Z0);
        Line2D(2+i) = cam.d*(line3d(i+dX)*line3d(Z0)-line3d(i)*line3d(dZ)) / (line3d(Z0)*(line3d(Z0)+line3d(dZ)));
    }
    realToImageLine(Line2D,cam);
    NormalizeLine(Line2D);
    return Line2D;
}
Vector7d line2dTo3d(const Vector4d &line, camera cam, double z = 5, double dz = 0){
    double Xs = cam.size.x/cam.pixel.x;
    double Ys = cam.size.y/cam.pixel.y;
    Vector7d line3d;
    line3d(0) = line(0)*Xs*z/cam.d;
    line3d(1) = line(1)*Ys*z/cam.d;
    line3d(2) = z;

    line3d(4) = line3d(0)*dz/z+line(2)*(z+dz)/cam.d;
    line3d(5) = line3d(1)*dz/z+line(3)*(z+dz)/cam.d;
    line3d(6) = dz;
    NormalizeLine(line3d);
    return line3d;
}


// ########################## ROS Helper Functions #################################
inspec_msg::line2d line2ros2D(lineEstimate line){
    inspec_msg::line2d ret;
    if(line.trust_estimate){
        ret = convert::line2ros(line.line2d);
    }else{
        ret = line.last_correct;
    } 
    
    ret.id = line.id;
    return ret;
}
inspec_msg::line3d line2ros3D(lineEstimate line){
    return convert::line2ros(line.X_hat,line.id);
}
//############################ Running the kalmanFilter  ################################################
void addNewLine(inspec_msg::line2d line2d){
    lineEstimate newLine;

    if(line2d.id != 0){
        newLine.id = line2d.id;
        if(line2d.id > size_t(max_line_id)) max_line_id = line2d.id;
    }else{
        newLine.id = ++max_line_id;
    }
    newLine.lastObserved = image_seq;
    newLine.consecutiveObservations++;
    newLine.P = P_initial_diag;
    newLine.line2d = convert::ros2line(line2d);
    newLine.X_hat = line2dTo3d(newLine.line2d,currentCam,5,0.3); //TODO correct this for better start guess
    ActiveLines[newLine.id] = newLine;
}
void removeLine(lineEstimate line){
    ActiveLines.erase(line.id);
}
void updateLineEstimate(lineEstimate &line, const Matrix7 &F){
    //    Xx_hat = Fx*Xx_hat+Gx*u(1);
    //    Px = Fx*Px*Fx'+Qx;
    line.X_hat = F*line.X_hat;
    line.P = F*line.P*F.transpose()+Q;
    NormalizeLine(line.X_hat);
    line.line2d = line3dTo2d(line.X_hat,currentCam);

}
void correctLineEstimate(lineEstimate &theLine, const inspec_msg::line2d &correction_data){ 
    if(image_seq - theLine.lastObserved == 1){
        theLine.consecutiveObservations++;
    }else{
        theLine.consecutiveObservations = 1;
    }
    theLine.observations++;
    theLine.lastObserved = image_seq;
    theLine.last_correct = correction_data;

    Vector4d Z = convert::ros2line(correction_data);
    Matrix4x7 H = H_matrix(theLine.X_hat,currentCam);
    const Matrix4 &R = Sigma_r_diag*H*H.transpose()*Sigma_r_diag.transpose();

    Matrix4 S = H*theLine.P*H.transpose()+R;
    Matrix7x4 K = theLine.P*H.transpose()*S.inverse();
    theLine.P = theLine.P - K*H*theLine.P;
    theLine.X_hat = theLine.X_hat + K*(Z-theLine.line2d);
    NormalizeLine(theLine.X_hat);

    theLine.line2d = line3dTo2d(theLine.X_hat,currentCam);
    if(math::lineError(theLine.line2d,Z) < MAX_ERROR_FOR_EST_MATCHING){
        theLine.trust_estimate = true;
    }else{
        theLine.trust_estimate = false;
    }
    //cout << "Line: " << theLine.id << '\t' << " - " << theLine.X_hat.transpose() << endl;
    cout << "Line: " << theLine.id << " - error: " << math::lineError(theLine.line2d,Z) << endl;

}
// ########################## ROS handlers ####################################

void line_handler(inspec_msg::line2d_array msg){   
    std::cout << "Image Data Recived: " << long(msg.header.seq) << endl;
    if(msg.header.seq > image_seq && msg.header.seq == position_seq){
        image_seq = msg.header.seq;
        inspec_msg::line3d_array return_msg;
        for(uint i = 0; i < msg.lines.size(); i++){
            try{
                lineEstimate &line = ActiveLines.at(size_t(msg.lines[i].id));
                correctLineEstimate(line,msg.lines[i]);
                if(line.trust_estimate){
                    return_msg.lines.push_back(line2ros3D(line));
                }
            }catch(const std::out_of_range& oor) {
                addNewLine(msg.lines[i]);
            }  
        }
        line_pub.publish(return_msg);
    }
}
void position_handler(inspec_msg::position msg){
    std::cout << "Position Recived: " << msg.header.seq << endl;
    if(msg.header.seq > position_seq){
        position_seq = msg.header.seq;
        positionQueue.push_front(msg);
        positionQueue.pop_back();

        Matrix7 F = F_matrix(   convert::FRU2Image3D(convert::ros2Vector3D(msg.position)),
                                convert::FRU2Image3D(convert::ros2Quaternion(msg.Orientation_quat)));

        inspec_msg::line2d_array return_msg;
        vector<lineEstimate> toRemove;

        for (pair<const long int,lineEstimate> &x : ActiveLines){
            if( position_seq-x.second.lastObserved > MAX_UNOBSERVED_STATES_BEFORE_DELETION ||
                (position_seq-x.second.lastObserved > MAX_UNOBSERVED_STATES_BEFORE_DELETION_UNTRUSTED_LINE
                 && x.second.observations < MIN_OBSERVATION_TOO_BEE_TRUSTED))
            {
                toRemove.push_back(x.second);
            }else{
                updateLineEstimate(x.second,F);
                return_msg.lines.push_back(line2ros2D(x.second));

            }      
        }
        for(uint i = 0; i < toRemove.size(); i++) removeLine(toRemove[i]);
        
        return_msg.header =msg.header;
        return_msg.header.stamp = ros::Time::now();

        line2Dest_pub.publish(return_msg);
    }
}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"estimator3d");
    ros::NodeHandle nh = ros::NodeHandle();

    settings::read(setting_camera);
 
    currentCam.pixel.x = setting_camera.pixel_width;
    currentCam.pixel.y = setting_camera.pixel_height;
    currentCam.d = setting_camera.d;
    currentCam.size.x = setting_camera.Chip_size_x;
    currentCam.size.y = setting_camera.Chip_size_y;

    // Initialize constatn Matrix;
    X_hat_initial_guess << 0,0,5,0,0,0,0;
    P_initial   << 3    ,3      ,10     ,0      ,0.2    ,0.2    ,0.2;
    Sigma_u     << 1    ,1      ,1      ,0      ,0.1    ,0.1    ,0.1;
    Sigma_r     << 0.1  ,0.1    ,0.01   ,0.01;
    for(int i = 0; i < 7; i++){
        G(i,i) = 1;
        if (i < 4) Sigma_r_diag(i,i) = Sigma_r(i);
        Sigma_u_diag(i,i) = Sigma_u(i);
        P_initial_diag(i,i) = P_initial(i);
    }
    Q = G*Sigma_u_diag*Sigma_u_diag.transpose()*G.transpose(); 
    // Initialize ROS publishers and Subscribers
    line_sub = nh.subscribe("/linedetector/lines2d",1,line_handler);
    position_sub = nh.subscribe("DroneInfo/Relative/Position",10,position_handler);
    line_pub = nh.advertise<inspec_msg::line3d_array>("/Estimator/lines3d",10);
    line2Dest_pub = nh.advertise<inspec_msg::line2d_array>("/Estimator/lines2d",10);
    
    //Let ros spin
    ros::spin();



    return 0;
}