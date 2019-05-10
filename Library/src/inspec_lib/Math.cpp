#include "../include/inspec_lib/Math.hpp"

namespace math{
    mathLine2d constructMathLine(cv::Point2f p, double angle, cv::Mat *img){
        mathLine2d line;
        line.a=atan(angle);
        line.b=p.y-p.x*line.a;
        if(img!= NULL){
            
            double y = line.a*img->cols/2+line.b;
            line.a *= -1;
            line.b = -(y-img->rows/2);
        }
        return line;
    }
    mathLine2d constructMathLine(cv::Point2f p1, cv::Point2f p2, cv::Mat *img){
        mathLine2d line;
        line.a= (p1.y-p2.y) / (p1.x-p2.x);
        line.b=p1.y-p1.x*line.a;

        if(img!= NULL){
            double y = line.a*img->cols/2+line.b;
            line.a *= -1;
            line.b = -(y-img->rows/2);
        }
        return line;
    }
    cv::Point2f intersection(mathLine2d l1, mathLine2d l2){
        cv::Point2f p;
        p.x = (l2.b-l1.b) / (l1.a-l2.a);
        p.y = l1.a * p.x + l1.b;
        return p;
    }

    // ####################### CONVERTERS ################################
    
    double rad2deg(const double &angle){
        return angle*180/M_PI;
    }
    double deg2rad(const double &angle){
        return angle*M_PI/180;
    }
    // ######################## ALGORITHMS ###############################
    mathLine2d leastSquareRegression(std::vector<cv::Point> aLine, const cv::Size &imgSize, bool move00toImgCenter){ // Should be const ref
        int N = aLine.size();
        double y_sum, x_sum, xy_sum, xx_sum;
        for(int i = 0; i < N; i++){
            double x,y;
            if(move00toImgCenter){
                x = aLine[i].x-imgSize.width/2;
                y = -(aLine[i].y-imgSize.height/2);    
            }else{
                x = aLine[i].x;
                y = aLine[i].y;
            }

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
    mathLine2d leastSquareRegression(std::vector<cv::Point2f> aLine, const cv::Size &imgSize, bool move00toImgCenter){ // Should be const ref
        int N = aLine.size();
        double y_sum, x_sum, xy_sum, xx_sum;
        for(int i = 0; i < N; i++){
            double x,y;
            if(move00toImgCenter){
                x = aLine[i].x-imgSize.width/2;
                y = -(aLine[i].y-imgSize.height/2);    
            }else{
                x = aLine[i].x;
                y = aLine[i].y;
            }

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
    double distance(const cv::Point &p1, const cv::Point p2){
        return std::sqrt(std::pow(p1.x-p2.x,2)+std::pow(p1.y-p2.y,2));
    }
    double distance(const cv::Point &p, const mathLine2d &l){
        return distance(l,p);
    }
    double distance(const mathLine2d l, const cv::Point p){
        double a = l.a;
        double c = l.b;
        double b = -1;

        return std::abs(a*p.x+b*p.y+c)/std::sqrt(a*a+b*b);
    }

    double dot(const rw::math::Vector3D<double> v1,const rw::math::Vector3D<double> v2){
        return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
    }
    // ######################## DEBUG #####################################
    void drawMathLine(cv::Mat &dst, mathLine2d line, cv::Scalar color,std::string text,cv::Scalar textColor){
        int x_max = dst.rows;
        int x_min = -1;
        for(int x = 0; x< dst.cols; x++){
            double b = (-line.b+dst.rows/2.0)-(-line.a)*(dst.cols/2.0);
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
