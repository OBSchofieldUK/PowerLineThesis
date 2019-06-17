/*
 * RoVi1
 * Example application to load and display an image
 */

// v1.0-4-gdfe246a

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <ctime>

using namespace std;

#define HORIZONTAL 255
#define VERTICAL 0

vector<cv::Point> rowMaximum(cv::Mat Gradient,cv::Mat Direction,int skip,uchar treshold = 36){
    cv::Point Max_p;
    vector<cv::Point> result;
    uchar* row_g;
    uchar* row_d;
    int value =0;
    for(int y = 1; y< Gradient.cols-1;y+= skip+1){
        row_g = Gradient.ptr<uchar>(y);
        row_d = Direction.ptr<uchar>(y);
        Max_p.y = y;
        for(int x = 1; x < Gradient.rows-1;x++){
            value = row_g[x];
            if(row_d[x] == VERTICAL){
                if(value-Gradient.ptr<uchar>(y-1)[x] >= treshold &&
                   value-Gradient.ptr<uchar>(y+1)[x] >= treshold)
                {
                    Max_p.x = x;
                    result.push_back(Max_p);
                    if( x == 212 && y == 265){
                        cout << "(" << int(Gradient.ptr<uchar>(y-1)[x]) << "," << Gradient.ptr<uchar>(y+1)[x] << ")" << endl;
                        cout << "(" << int(Gradient.ptr<uchar>(y)[x-1]) << "," << Gradient.ptr<uchar>(y)[x+1] << ")" << endl;
                    }
                }
            }else{
                if(value-row_g[x-1]>=treshold &&
                   value-row_g[x+1]>=treshold)
                {
                    if( x == 212 && y == 265){
                        cout << "(" << int(Gradient.ptr<uchar>(y-1)[x]) << "," << Gradient.ptr<uchar>(y+1)[x] << ")" << endl;
                        cout << "(" << int(Gradient.ptr<uchar>(y)[x-1]) << "," << Gradient.ptr<uchar>(y)[x+1] << ")" << endl;
                    }

                    Max_p.x = x;
                    result.push_back(Max_p);
                }
            }
        }
    }
    return result;
}

void EdgeDraw(const cv::Mat& src, cv::Mat& dst, uchar skip = 4,uchar edge_treshold=8, uchar treshold = 36){
    cv::Mat gray,grad_x,grad_y;

    //########### Get Gradient MAP ##############
    cvtColor(src,gray,CV_BGR2GRAY);
    GaussianBlur(gray,gray,cv::Size(5,5),1);
    Sobel(gray,grad_x,CV_16S,1,0);
    Sobel(gray,grad_y,CV_16S,0,1);

    cv::Mat abs_x,abs_y,grad;
    convertScaleAbs(grad_x,abs_x);
    convertScaleAbs(grad_y,abs_y);
    addWeighted(abs_x,0.5,abs_y,0.5,0,grad);

    //########### Get Gradien Direction ###########
    cv::Mat Direction = abs_x>=abs_y;

    //########### Treshold Image ##################

    threshold(grad,grad,36,255,3);

    //########### Find Anchor Points ##############
    vector<cv::Point> max = rowMaximum(grad,Direction,3,8);


    //########### Trace Lines
    cv::Mat EdgeMap = cv::Mat::zeros(grad.rows,grad.cols,CV_8U);

    for(int i = 0; i <max.size();i++){
        cv::Point p = max[i];

        cv::waitKey(1);


        if(Direction.at<uchar>(max[i])==VERTICAL){
            while(grad.at<uchar>(p)>0 &&
                  Direction.at<uchar>(p) == VERTICAL &&
                  !EdgeMap.at<uchar>(p))
            {
                EdgeMap.at<uchar>(p)=255;
                uchar U,D,M;
                U= grad.at<uchar>(p+cv::Point(-1,+1));
                D= grad.at<uchar>(p+cv::Point(-1,-1));
                M= grad.at<uchar>(p+cv::Point(-1,0));
                if(D>M && D>U){
                    p.y -=1;
                }else if(U>M && U >D){
                    p.y +=1;
                }
                p.x -=1;
            }
            if(grad.at<uchar>(p)>0 && !EdgeMap.at<uchar>(p)){
                max.push_back(p);
            }
            p = max[i];
            while(grad.at<uchar>(p)>0 &&
                  Direction.at<uchar>(p) == VERTICAL &&
                  !EdgeMap.at<uchar>(p))
            {
                EdgeMap.at<uchar>(p)=255;
                uchar U,D,M;
                U= grad.at<uchar>(p+cv::Point(+1,+1));
                D= grad.at<uchar>(p+cv::Point(+1,-1));
                M= grad.at<uchar>(p+cv::Point(+1,0));
                if(D>M && D>U){
                    p.y -=1;
                }else if(U>M && U >D){
                    p.y +=1;
                }
                p.x +=1;
            }
            if(grad.at<uchar>(p)>0 && !EdgeMap.at<uchar>(p)){
                max.push_back(p);
            }
        }else{
            // ##### FOLLOW LINE UP #####
            while(grad.at<uchar>(p)>0 &&
                  Direction.at<uchar>(p) == HORIZONTAL &&
                  !EdgeMap.at<uchar>(p))
            {
                EdgeMap.at<uchar>(p)=255;
                uchar U,D,M;
                U= grad.at<uchar>(p+cv::Point(+1,-1));
                D= grad.at<uchar>(p+cv::Point(-1,-1));
                M= grad.at<uchar>(p+cv::Point(0,-1));
                if(D>M && D>U){
                    p.x -=1;
                }else if(U>M && U >D){
                    p.x +=1;
                }
                p.y -=1;
            }
            if(grad.at<uchar>(p)>0 && !EdgeMap.at<uchar>(p)){
                max.push_back(p);
            }

            // ##### FOLLOW LINE DOWN #####
            p = max[i];
            while(grad.at<uchar>(p)>0 &&
                  Direction.at<uchar>(p) == VERTICAL &&
                  !EdgeMap.at<uchar>(p))
            {
                EdgeMap.at<uchar>(p)=255;
                uchar U,D,M;
                U= grad.at<uchar>(p+cv::Point(+1,+1));
                D= grad.at<uchar>(p+cv::Point(-1,+1));
                M= grad.at<uchar>(p+cv::Point(0,+1));
                if(D>M && D>U){
                    p.x -=1;
                }else if(U>M && U >D){
                    p.x +=1;
                }
                p.y +=1;
            }
            if(grad.at<uchar>(p)>0 && !EdgeMap.at<uchar>(p)){
                max.push_back(p);
            }
        }
    }
    dst = EdgeMap;
}

int main(int argc, char* argv[])
{
    // Parse command line arguments
    cv::CommandLineParser parser(argc, argv,
        "{help   |            | print this message}"
        "{@image | ../Lena.png | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image
    std::string filename = parser.get<std::string>("@image");
    cv::Mat img = cv::imread(filename);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }



    clock_t begin = clock();
    cv::Mat edge;
    for(uint i = 0; i < 1000;i++){
        
        EdgeDraw(img,edge,4,4);
    }

    clock_t end = clock();

    double elapsed = (1000*double(end - begin) / CLOCKS_PER_SEC)/1000;
    cout << "Edge draw: " << elapsed << "ms" << endl;
    cv::imshow("Edge",edge);





    begin = clock();
    cv::Mat detected_edges,src_gray;
    for(uint i = 0; i < 1000;i++){
        
        cvtColor(img,src_gray,CV_BGR2GRAY);

        blur( src_gray, detected_edges, cv::Size(3,3) );
        Canny( detected_edges, detected_edges, 36, 150, 3);
    }

    end = clock();

    elapsed = (1000*double(end - begin) / CLOCKS_PER_SEC)/1000;
    cout << "canny: " << elapsed << "ms" << endl;
    imshow( "canny", detected_edges );


    cv::waitKey();

    return 0;
}
