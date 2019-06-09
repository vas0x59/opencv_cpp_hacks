#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
// #include "search_sq.h"
// #include "optical_flow_func.h"
using namespace std;
using namespace cv;

class Test_cpp_node
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher point_pub;
    ros::Subscriber id_sub;
  public:
    // bool prev_e_;
    // int prev_s_;
    // int prev_x_;
    // int prev_y_;
    Mat res, fr_now, fr_prev;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // cv::aruco::detectMarkers(image, dictionary, corners, ids);
    Point2f out_x_y;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    int id_i = 2;
    //   vector<Point2f> p_prev, p_now;
    //   vector<float> flow_errs;
    //   vector<unsigned char> flow_status;
    //   bool first_run = true;
    Test_cpp_node()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/main_camera/image_raw/throttled", 1,
                                   &Test_cpp_node::imageCb, this);
                    
        image_pub_ = it_.advertise("/Test_cpp_node/debug", 1);
        point_pub = nh_.advertise<geometry_msgs::Point>("/point_vision", 10);
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        id_sub = nh_.subscribe("/id_i", 10, &Test_cpp_node::id_cb, this);
        // int id_i = 1;
        //cv::namedWindow(OPENCV_WINDOW);
    }

    ~Test_cpp_node()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }
    void id_cb(const std_msgs::Int8::ConstPtr &msg){
        id_i = msg->data;
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        cv::flip( cv_ptr->image,  cv_ptr->image, -1);
        res = cv_ptr->image;
        // cvtColor(fr_prev, fr_prev, cv::COLOR_BGR2GRAY);
        cvtColor(cv_ptr->image, fr_now, cv::COLOR_BGR2GRAY);
        cv::aruco::detectMarkers(fr_now, dictionary, corners, ids);
        // if at least one marker detected
        bool est_mark = false;
        if (ids.size() > 0){
            cv::aruco::drawDetectedMarkers(res, corners, ids);
            for (size_t i =0; i < ids.size(); i++){
                if (ids[i] == id_i){
                    // cout << corners[i];
                    out_x_y = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) / 4;
                    est_mark = true;
                }
            }
        }
        // cout <<out_x_y;
        geometry_msgs::Point point_msg;
        if (est_mark){
            point_msg.x = out_x_y.x;
            point_msg.y = out_x_y.y;
            point_msg.z = 0;
        }
        else{
            point_msg.x = -1;
            point_msg.y = -1;
            point_msg.z = -1;
        }
        point_pub.publish(point_msg);
        circle(res, out_x_y, 3, Scalar(255, 0, 0), -1, 8);
        // line(res, out_x_y, Point(160, 120), Scalar(0, 225, 255), -1, 
        line(res, out_x_y, Point(160, 120), Scalar(0, 225, 255), 3, LINE_AA);
        cv_ptr->image = res;
        image_pub_.publish(cv_ptr->toImageMsg());
        
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    Test_cpp_node ic;
    ros::Rate loop_rate(10);
    while (true)
    {
        // cout << ic.prev_x_ << endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
