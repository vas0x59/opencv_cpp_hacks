#include <iostream>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "math.h"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"

// #include "fast_line_detector.hpp"
using namespace std;
using namespace cv;

int MAX_POINTS = 75;

// Matx33d H;
double H_data[9] = {6.3634226941243310e+02, 0, 3.0857544996314618e+02, 0,
       6.3634226941243310e+02, 2.2963570168662270e+02, 0, 0, 1};
// double H_data[9] = { 8.3075929814809172e+02, 0, 3.2641209544757311e+02, 0,
//        8.3075929814809172e+02, 2.3133591986761215e+02, 0, 0, 1 };

Matx33d H = Mat(3, 3, CV_64F, H_data);

Point3f estimate_motion(vector<Point2f> &prev_coords, vector<Point2f> &curr_coords)
{ // function to calculate the rotational & translational motion from frame-to-frame

    int N = prev_coords.size();
    vector<Point3f> curr_homog;
    vector<Point3f> prev_homog;
    convertPointsToHomogeneous(curr_coords, curr_homog);
    convertPointsToHomogeneous(prev_coords, prev_homog);

    Point3f motion_avg = Point3f(0.0, 0.0, 0.0);

    // calc total left/right tracked point movement
    vector<Point3f>::iterator it1 = prev_homog.begin(); // sizes should already
    vector<Point3f>::iterator it2 = curr_homog.begin(); // be verified equal
    for (; it2 != curr_homog.end(); ++it1, ++it2)
    {                                                               // calculates robot coordinates from camera coordinates
        motion_avg += Point3f((H * Mat(*it2) - H * Mat(*it1)) / N); // you can tell it's the average by the way that it is!
    }

    // correct y-axis (fore-aft) sign (so forward motion = positive value)
    // factor for speed of 0.1 (from spreadsheet)
    // motion_avg.y *= -1.57 * 4.0 / 3.0;

    // now apply deadband of 0.5 mm or so (so we don't accrue unnecessary noise errors)
    if (fabs(motion_avg.x) > 0.0005 || fabs(motion_avg.y) > 0.0005)
    {
        return motion_avg;
    }
    else
    {
        return Point3f(0.0, 0.0, 0.0);
    }
}

int main(int argc, char **argv)
{
    VideoCapture cap(1); // open the default camera
    if (!cap.isOpened()) // check if we succeeded
        return -1;

    vector<Point2f> p_prev, p_now;
    vector<float> flow_errs;
    vector<unsigned char> flow_status;
    Mat fr_now, fr_prev, res, gray, now_img;

    cap >> fr_prev;
    cap >> fr_now;
    cvtColor(fr_prev, fr_prev, cv::COLOR_BGR2GRAY);
    cvtColor(fr_now, fr_now, cv::COLOR_BGR2GRAY);
    // goodFeaturesToTrack(fr_prev, p_prev, MAX_POINTS, 0.1, 5.0);

    // buildOpticalFlowPyramid(fr_prev, p_prev, Size(21, 21), 4, true);
    // buildOpticalFlowPyramid(fr_now, p_now, Size(21, 21), 4, true);
    while (true)
    {
        cap >> now_img;
        res = now_img;
        cvtColor(now_img, fr_now, cv::COLOR_BGR2GRAY);
        imshow("fr_now", fr_now);
        cout << p_prev.size() << "   " << p_now.size() << endl;
        if (p_prev.size() < 0.75 * MAX_POINTS) // if enough tracking indices are dropped, calculate a new set
        {
            // create vector of good points to track from previous image
            goodFeaturesToTrack(fr_prev, p_prev, MAX_POINTS, 0.1, 5.0);
            // p_now.resize(p_prev.size());
        }
        if (p_prev.size() != 0 || p_now.size() != 0)
        {
            calcOpticalFlowPyrLK(fr_prev, fr_now, p_prev, p_now, flow_status, flow_errs, Size(21, 21), 4);
            Point3d ep = estimate_motion(p_prev, p_now);

            cout << ep << endl;
            size_t i, k;
            // for (i = k = 0; i < p_now.size(); i++)
            // {
            //     // p_now[k++] = p_now[i];
            //     circle(res, p_now[i], 3, Scalar(0, 255, 0), -1, 8);
            // }
            for (i = k = 0; i < p_prev.size(); i++)
            {
                // p_prev[k++] = p_prev[i];
                circle(res, p_prev[i], 3, Scalar(0, 0, 255), -1, 8);
            }

            for (i = k = 0; i < p_now.size(); i++)
            {
                // if (addRemovePt)
                // {
                //     if (norm(point - points[1][i]) <= 5)
                //     {
                //         addRemovePt = false;
                //         continue;
                //     }
                // }
                if (!flow_status[i])
                    continue;
                p_now[k++] = p_now[i];
                circle(res, p_now[i], 3, Scalar(255, 0, 0), -1, 8);
            }
            p_now.resize(k);

            // p_now.resize(k);
            // p_prev = p_now;
            // fr_prev = fr_now;
            // string strr = to_string(ep.x * 10);
            // strr += " ";
            // strr += to_string(ep.y * 0.0);
            // strr += " ";
            // strr += to_string(ep.z * 0.0);
            // putText(res, strr, Point(0, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 0, 200), 1, cv::LINE_AA);
            putText(res, to_string(ep.x / 100000), Point(0, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 0, 200), 1, cv::LINE_AA);
            putText(res, to_string(ep.y / 100000), Point(0, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 0, 200), 1, cv::LINE_AA);
            putText(res, to_string(ep.z / 100000), Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 0, 200), 1, cv::LINE_AA);
            p_prev.resize(p_now.size());
            std::swap(p_prev, p_now);
        }
        imshow("res", res);

        // p_now.resize(p_prev.size());
        cv::swap(fr_prev, fr_now);
        waitKey(1);
    }
}