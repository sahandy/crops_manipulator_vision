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

// Helper typedefs to make the implementation code cleaner
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef typename PointCloudT::Ptr PointCloudPtr;
typedef typename PointCloudT::ConstPtr PointCloudConstPtr;

typedef pcl::PointXYZHSV PointHSV;
typedef pcl::PointCloud<PointHSV> PointCloudHSV;
typedef typename PointCloudHSV::Ptr PointCloudHSVPtr;
typedef typename PointCloudHSV::ConstPtr PointCloudHSVConstPtr;

class ImageConverter
{
public:
    ImageConverter()
            : it_(nh_) {
      // Subscrive to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                                 &ImageConverter::image_cb_, this);
      hsv_pub_ = it_.advertise("crops/vision/image_filter/hsv_filtered", 1);
    }

    ~ImageConverter() {}

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
      HSVFilter hsvFilter(cv_ptr->image);
      // TODO remove hard-coded color value
      hsvFilter.setColorValues("RED");
      cv::Mat thresholdImage = hsvFilter.getFilteredImage();
      hsvFilter.morphOps(thresholdImage);
      // Output modified video stream
      cv_ptr->image = thresholdImage;
      cv_ptr->encoding = sensor_msgs::image_encodings::MONO8; // output is a single-channel image
      hsv_pub_.publish(cv_ptr->toImageMsg());
    }
private:
    // Ros Node Handle
    ros::NodeHandle nh_;
    // Instance which is capable of receiving RGB image
    image_transport::ImageTransport it_;
    // Subscriber to the incoming RGB image (object received via ImageTransport
    image_transport::Subscriber image_sub_;
    // HSV-thresholded image Publisher
    image_transport::Publisher hsv_pub_;
};

int main (int argc, char **argv) {
  // Initialize ROS
  ros::init (argc, argv, "color_filter");
  // Receive and process (filter) incoming rgb image
  ImageConverter ic;
  // Spin
  ros::spin ();

  return 0;
}