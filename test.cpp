#include "opencv2/opencv.hpp"
#include "search_sq.h"
using namespace cv;

bool prev_e;
int prev_s;
int prev_x;
int prev_y;

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat frame, res;
    namedWindow("frame",1);
    namedWindow("res",1);
    cout << "Hello world" << endl;
    for(;;)
    {
        
        //Mat res;
        cap >> frame; // get a new frame from camera
        res = frame;
        /*cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        */
        imshow("frame", frame);
        
        bool q = search_sq(frame, RED_COLOR_HSV, prev_e,  prev_s, prev_x, prev_y, res);
        imshow("res", res);
        //cout << prev_s << endl;
        if(waitKey(2) == 1) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
