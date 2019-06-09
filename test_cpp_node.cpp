#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"
// #include "search_sq.h"
#include "optical_flow_func.h"
using namespace std;
using namespace cv;

class Test_cpp_node
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  // bool prev_e_;
  // int prev_s_;
  // int prev_x_;
  // int prev_y_;
  Mat res, fr_now, fr_prev;
  vector<Point2f> p_prev, p_now;
  vector<float> flow_errs;
  vector<unsigned char> flow_status;
  bool first_run = true;
  Test_cpp_node()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/main_camera/image_raw/throttled", 1,
      &Test_cpp_node::imageCb, this);
    image_pub_ = it_.advertise("/Test_cpp_node/debug", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~Test_cpp_node()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
  
    res = cv_ptr->image;
    // cvtColor(fr_prev, fr_prev, cv::COLOR_BGR2GRAY);
    cvtColor(cv_ptr->image, fr_now, cv::COLOR_BGR2GRAY);
    if (first_run == true){
      cv::swap(fr_prev, fr_now);
      first_run == false;
    }
    // bool q = search_sq(cv_ptr->image, RED_COLOR_HSV, prev_e_,  prev_s_, 
    //   prev_x_, prev_y_, res_);
    Point3d opf = optical_flow(fr_now, fr_prev, p_now, p_prev, flow_status, flow_errs, res);
    cout << opf;
    cv_ptr->image = res;
    //imshow("res", res);
    //cout << prev_s << endl;
    // waitKey(1);
    // Update GUI Window

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    p_prev.resize(p_now.size());
    std::swap(p_prev, p_now);
    cv::swap(fr_prev, fr_now);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  Test_cpp_node ic;
  ros::Rate loop_rate(10);
  while (true){
    // cout << ic.prev_x_ << endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
