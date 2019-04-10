#include "../include/inspec_lib/Math.hpp"

namespace math{

    // ####################### CONVERTERS ################################
    
    double rad2deg(const double &angle){
        return angle*180/M_PI;
    }
    double deg2rad(const double &angle){
        return angle*M_PI/180;
    }
    // ######################## ALGORITHMS ###############################
    mathLine2d leastSquareRegression(std::vector<cv::Point> aLine, const cv::Size &imgSize){ // Should be const ref
        int N = aLine.size();
        double y_sum, x_sum, xy_sum, xx_sum;
        for(int i = 0; i < N; i++){
            const double x = aLine[i].x-imgSize.width/2;
            const double y = -(aLine[i].y-imgSize.height/2);

            y_sum += y;
            x_sum += x;
            xy_sum += x*y;
            xx_sum += x*x;
        }

        mathLine2d ret;
        ret.a = (N*xy_sum-x_sum*y_sum)/(N*xx_sum-(x_sum*x_sum));
        ret.b = (y_sum-ret.a*x_sum)/N;
        return ret;
    }
    double vecAngle(const inspec_msg::line2d &l1, const inspec_msg::line2d &l2){
        double length1 = std::sqrt(std::pow(l1.dx,2)+std::pow(l1.dy,2));
        double length2 = std::sqrt(std::pow(l2.dx,2)+std::pow(l2.dy,2));
        double dot = l1.dx*l2.dx+l1.dy*l2.dy;
        double angle = dot/(length1*length2);
        return std::abs(std::acos(angle));
    }
    
    double lineError(const inspec_msg::line2d &l1, const inspec_msg::line2d &l2){
        return lineError(
                vecAngle(l1,l2),
                convert::ros2mathLine(l1).b - convert::ros2mathLine(l2).b
            );
    }
    double lineError(Vector4d l1, Vector4d l2){
        return lineError(convert::line2ros(l1),convert::line2ros(l2));
    }
    double lineError(mathLine2d l1, mathLine2d l2){
        return lineError(
                vecAngle(convert::mathLine2ros(l1),convert::mathLine2ros(l2)),
                l1.b-l2.b
            );
    }
    double lineError(double angle, double offset){
        return (offset/5)*(1+int(offset/100))+angle*180/M_PI;
    }
    
    // ######################## DEBUG #####################################
    void drawMathLine(cv::Mat &dst, mathLine2d line, cv::Scalar color,std::string text,cv::Scalar textColor){
        int x_max = dst.rows;
        int x_min = -1;
        for(int x = 0; x< dst.cols; x++){
            double b = (-line.b+dst.rows/2)-(-line.a)*(dst.cols/2);
            double a = -line.a;
            int y = a*x+b;
            if(y > 0 && y < dst.rows){
                dst.at<cv::Vec3b>(cv::Point(x,y)) = {uchar(color[0]),uchar(color[1]),uchar(color[2])};
                if(x_min == -1) x_min = x;
                
            }else if(x_max == dst.rows && x_min != -1){
                x_max=x-1;
            }
        }
        if(!text.empty() && x_min != -1){
            double b = (-line.b+dst.rows/2)-(-line.a)*(dst.cols/2);
            double a = -line.a;
            double x = x_min+(x_max-x_min) / (1080/b);
            int y = a*x+b;

            cv::putText(dst,text,cv::Point(x,y),cv::FONT_HERSHEY_SIMPLEX,1,textColor,2);
        }
    }
    std::ostream& operator<<(std::ostream& os, const mathLine2d& dt){
        os << "y = " << dt.a << "\t * x + " << dt.b ;
        return os;
    }
    std::ostream& operator<<(std::ostream& os, const inspec_msg::line2d& dt){
        os << "line: " << dt.id << "pos(" << dt.x0 << ", " << dt.y0 << ") dir(" << dt.dx << ", " << dt.dy << ")";
        return os;
    }
}
