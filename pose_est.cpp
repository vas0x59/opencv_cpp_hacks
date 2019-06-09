#include <iostream>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "math.h"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"

// #include "fast_line_detector.hpp"
using namespace std;
using namespace cv;

Point3d rotationMatrixToEAngles(Mat R)
{
    double sy = sqrtf(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (not singular)
    {
        x = atan2f(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = atan2f(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else {
        x = atan2f(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Point3d(x, y, z);
}

int main(int argc, char **argv)
{
    FileStorage fs2("./logitech.yml", FileStorage::READ);
    Mat cameraMatrix, distCoeffs;
    fs2["cameraMatrix"] >> cameraMatrix;
    fs2["distCoeffs"] >> distCoeffs;
    fs2.release();

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    Mat _R_flip = Mat(3, 3, CV_64F, cvScalar(0.));
    _R_flip.at<double>(0,0) = 1.0;
    _R_flip.at<double>(1,1) = -1.0;
    _R_flip.at<double>(2,2) = -1.0;
    cv::VideoCapture inputVideo;
    

    int id_i = 1;
    int cam_id = 1;
    float marker_size = 0.1;
    cout << "m_s: ";
    cin >> marker_size;
    cout << "id: ";
    cin >> id_i;
    cout << "cam_id: ";
    cin >> cam_id;
    inputVideo.open(cam_id);
    while (inputVideo.grab())
    {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        // cv::aruco::detectMarkers(image, dictionary, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        if (ids.size() > 0)
        {
            // cv::aruco::estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);
        
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            for (size_t i = 0; i < ids.size(); i++)
            {
                if (ids[i] == id_i)
                {
                    cv::Vec3d tvec, rvec;
                    Mat pos_camera = Mat(3, 3, CV_64F, cvScalar(0.));
                    Mat R_ct, R_tc;
                    // tvec = tvecs[i];
                    // rvec = rvecs[i];
                    cout << "pose_raw  x:" << tvec[0] << " y:" << tvec[1] << " z:" << tvec[2] << endl;
                    cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
                    cv::Rodrigues(rvec, R_ct);
                    R_tc = R_ct.t();
                    double roll_marker, pitch_marker, yaw_marker;
                    Point3d marker_angles;
                    marker_angles = rotationMatrixToEAngles(_R_flip*R_tc);
                    roll_marker = marker_angles.x;
                    pitch_marker = marker_angles.y;
                    yaw_marker = marker_angles.z;
                    cout << "marker_angles_raw  roll:" << roll_marker << " pitch:" << pitch_marker << " yaw:" << yaw_marker << endl;
                    Mat tvec_mat = Mat(tvec);
                    R_tc = -R_tc;
                    pos_camera = R_tc*tvec_mat;
                    cout << "pose  x:" << pos_camera.at<double>(0, 0) << " y:" << pos_camera.at<double>(0,1)<< " z:" << pos_camera.at<double>(0,2)<< endl;
                    // cout << corners[i];
                }
            }
        }
        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(1);
        if (key == 27)
            break;
    }
    return 0;
}