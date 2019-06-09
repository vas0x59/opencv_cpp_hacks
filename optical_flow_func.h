#include <iostream>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "math.h"

using namespace std;
using namespace cv;

#define MAX_POINTS 100

double H_data[9] = {8.2381101021315180e+02, 0, 3.1099830580461673e+02, 0,
                    8.2381101021315180e+02, 2.2482767964412707e+02, 0, 0, 1};
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

Point3d optical_flow(Mat &fr_now, Mat &fr_prev,
                     vector<Point2f> &p_now, vector<Point2f> &p_prev,
                     vector<unsigned char> &flow_status,
                     vector<float> flow_errs, Mat &res)
{
    Point3d ep;
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
        ep = estimate_motion(p_prev, p_now);

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
    }
    return ep;
}