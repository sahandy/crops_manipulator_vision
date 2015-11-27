//
// Created by sahand on 23.11.15.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>

#include <opencv2/highgui/highgui.hpp>

#include "HSVFilter.hpp"


// Subscriber to the incoming RGB image (object received via ImageTransport
image_transport::Subscriber image_sub_;
// HSV-thresholded image Publisher
image_transport::Publisher hsv_pub_;
HSVFilter hsv_filter_;
int h_min_ = 12;
int h_max_ = 22;
int s_min_ = 130;
int s_max_ = 255;
int v_min_ = 140;
int v_max_ = 255;
int er_sz_ = 1, dl_sz_ = 3;
int er_it_ = 1, dl_it_ = 1;

void on_trackbar(int, void*) {
  hsv_filter_.setColorValues(h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);
  hsv_filter_.setMorphKernels(er_sz_, dl_sz_);
  hsv_filter_.setMorphIters(er_it_, dl_it_);
}

void image_cb_(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // HSV filtering based on the input color
  hsv_filter_.setImage(cv_ptr->image);
  //hsv_filter_.setImage(cv_ptr->image);
  // TODO remove hard-coded color value
  // hsvFilter.setColorValues("RED");
  cv::Mat thresholdImage = hsv_filter_.getFilteredImage();
  hsv_filter_.morphOps(thresholdImage);
  // Output modified video stream
  cv_ptr->image = thresholdImage;
  cv_ptr->encoding = sensor_msgs::image_encodings::MONO8; // output is a single-channel image
  hsv_pub_.publish(cv_ptr->toImageMsg());
  cv::waitKey(30);
}

int main (int argc, char **argv) {
  // Initialize ROS
  ros::init (argc, argv, "color_filter");

  // Ros Node Handle
  ros::NodeHandle nh_;
  // Instance which is capable of receiving RGB image
  image_transport::ImageTransport it_(nh_);
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, image_cb_);
  hsv_pub_ = it_.advertise("crops/vision/image_filter/hsv_filtered", 1);

  std::cout << "interactive mode: ON" << std::endl;
  // Setup the trackbars
  cv::namedWindow("HSV Range Controller", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("H_MIN", "HSV Range Controller", &h_min_, 179, on_trackbar);
  cv::createTrackbar("H_MAX", "HSV Range Controller", &h_max_, 179, on_trackbar);
  cv::createTrackbar("S_MIN(%)", "HSV Range Controller", &s_min_, 255, on_trackbar);
  cv::createTrackbar("S_MAX(%)", "HSV Range Controller", &s_max_, 255, on_trackbar);
  cv::createTrackbar("V_MIN(%)", "HSV Range Controller", &v_min_, 255, on_trackbar);
  cv::createTrackbar("V_MAX(%)", "HSV Range Controller", &v_max_, 255, on_trackbar);
  cv::createTrackbar("Erosion Kernel\nSize: 2n+1", "HSV Range Controller", &er_sz_, 21, on_trackbar);
  cv::createTrackbar("Erosion Iterations", "HSV Range Controller", &er_it_, 5, on_trackbar);
  cv::createTrackbar("Dilation Kernel\nSize: 2n+1", "HSV Range Controller", &dl_sz_, 21, on_trackbar);
  cv::createTrackbar("Dilation Iterations", "HSV Range Controller", &dl_it_, 5, on_trackbar);
  // Initialize trackbars
  on_trackbar(0, 0);
  // Spin
  ros::spin ();

  return 0;
}
