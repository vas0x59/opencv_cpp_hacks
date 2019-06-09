#include "opencv2/opencv.hpp" 
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

Scalar RED_COLOR_HSV[] = {Scalar(0, 106, 90), Scalar(255, 255, 255)};
Scalar GREEN_COLOR_HSV[] = {Scalar(32, 48, 69), Scalar(76, 164, 239)};


bool search_sq(Mat frame, Scalar* color, bool &prev_e,  int &prev_s, int &prev_x, int &prev_y, Mat &res){
    Mat hsv, thresh;
    cvtColor(frame, hsv, cv::COLOR_RGB2HSV);
    inRange(hsv,color[0], color[1],thresh);
    medianBlur ( thresh, thresh, 5 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    findContours( thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    int x = 0;
    int y = 0;
    for( size_t i = 0; i< contours.size(); i++ )
    {
        if (contourArea(contours[i]) > 200){
            RotatedRect rect = minAreaRect( contours[i] );
            //Scalar draw_color = Scalar( 0, 255, 0);
            Point2f rect_points[4]; 
            rect.points( rect_points );
            
            //boxPoints(rect, points);
            int w = rect.size.width;
            int h = rect.size.height;
            x = rect.center.x;
            y = rect.center.y;
            if ((w / h) < 1.8){
                cout << "x:" << x << " y:" << y << endl;
                for( int j = 0; j < 4; j++ )
                    line( res, rect_points[j], rect_points[(j+1)%4], Scalar( 0, 255, 0), 2, 8 );
            
                drawContours( res, contours, (int)i, Scalar( 0, 0, 110), 2, 8, hierarchy, 0, Point() );
            }
        }
    }
    prev_s = 80;
    prev_x = x;
    prev_y = y;
    return true;
}
