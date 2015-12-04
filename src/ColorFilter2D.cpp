//
// Created by sahand on 23.11.15.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>

#include <opencv2/highgui/highgui.hpp>

#include "HSVFilter.hpp"

// Subscriber to the incoming RGB image (object received via ImageTransport
image_transport::Subscriber image_sub_;
// HSV-thresholded image Publisher
image_transport::Publisher hsv_pub_;
// subscriber to receive HSV Interactive mode state
ros::Subscriber hsvInteractive_sub_;
// subscriber to receive HSV Values from GUI
ros::Subscriber hsvValue_sub_;

bool interactive_ = false;

HSVFilter hsv_filter_;

// callbacks
void image_cb_(const sensor_msgs::ImageConstPtr& msg);
void set_interactive_(const std_msgs::Int32::ConstPtr& msg);
void set_hsv_values_(const std_msgs::Int32MultiArray::ConstPtr& msg);


int main (int argc, char **argv) {
  // Initialize ROS
  ros::init (argc, argv, "color_filter");
  // ROS Node Handle
  ros::NodeHandle nh_;
  // Instance which is capable of receiving RGB image
  image_transport::ImageTransport it_(nh_);
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, image_cb_);
  hsvInteractive_sub_ = nh_.subscribe("toggle_hsv_mode", 1, set_interactive_);
  hsvValue_sub_ = nh_.subscribe("hsv_values", 10, set_hsv_values_);

  hsv_pub_ = it_.advertise("crops/vision/image_filter/hsv_filtered", 1);

  ros::spin();
  return 0;
}

/**
 * Callback that receives the message to toggle HSV interactive mode
*/
void set_interactive_(const std_msgs::Int32::ConstPtr& msg) {
  int state = msg->data;
  interactive_ = (state != 0);
  std::cout << "interactive mode: " << interactive_ << std::endl;
  // if(interactive_)
  //   create_control_panel();
  // else
  //   destroy_control_panel();
}

/**
 * Callback that receives the message containing new HSV filter values.
 * CAUTION: order of attributes in the message
 *        0. Hue_Min
 *        1. Hue_Max
 *        2. Saturation_Min
 *        3. Saturation_Max
 *        4. Value_Min
 *        5. Value_Max
 *        6. Erosion_Size
 *        7. Erosion_Iterations
 *        8. Dilation_Size
 *        9. Dilation_Iterations
 */
void set_hsv_values_(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  hsv_filter_.setHsvValues(
    msg->data[0], msg->data[1], msg->data[2],
    msg->data[3], msg->data[4], msg->data[5]);
  hsv_filter_.setMorphKernels(msg->data[6], msg->data[8]);
  hsv_filter_.setMorphIters(msg->data[7], msg->data[9]);
}

/**
 * Callback that receives the message containing the rgb iamge
*/
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
  cv::Mat thresholdImage = hsv_filter_.getFiltered();
  // cv::Mat thresholdImage = hsv_filter_.getFilteredImage(crops_vision::YELLOW);
  hsv_filter_.morphOps(thresholdImage);
  // Output modified video stream
  cv_ptr->image = thresholdImage;
  cv_ptr->encoding = sensor_msgs::image_encodings::MONO8; // output is a single-channel image
  hsv_pub_.publish(cv_ptr->toImageMsg());
  cv::waitKey(30);
}
