#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "types.h"

ros::Publisher pub;
image_transport::Subscriber image_sub_;

PointCloudCTPtr cloud;
cv::Mat threshold;

bool im, cb;

// Forward decleration
void ignite();
/**
 * Callback that receives the incoming point cloud
 */
void cloud_cb_ (const PointCloudCTConstPtr& cloud_msg);
/**
 * Callback that receives the incoming color-filtered image
 */
void image_cb_ (const sensor_msgs::ImageConstPtr& msg);

// Main
int main (int argc, char** argv) {
  im = false;
  cb = false;
  // Initialize ROS
  ros::init (argc, argv, "cloud_color_filter");
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_(nh_);
  // reading subscriber topic from cmd (if available)
  std::string in_cloud_topic;
  if(argc > 1)
    in_cloud_topic = argv[1];
  else
    in_cloud_topic = "/camera/depth_registered/points";
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub =
    nh_.subscribe (in_cloud_topic, 1, cloud_cb_);
  // Create a ROS subscriber for the input color filtered image
  image_sub_ = it_.subscribe("crops/vision/image_filter/hsv_filtered", 1, image_cb_);
  // Create a ROS publisher for the output model coefficients
  pub = nh_.advertise<PointCloudCT> ("/crops/vision/pointcloud_color_filtered", 1);
  // Spin
  ros::spin ();

  return 0;
}

void cloud_cb_ (const PointCloudCTConstPtr& cloud_msg) {
  cloud.reset(new PointCloudCT);
  pcl::copyPointCloud(*cloud_msg, *cloud);
  // set the cloud trigger to true
  cb = true;
  // perform 2d-3d correspondence extraction
  ignite();
}

void image_cb_ (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    threshold = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // set the image trigger to true
  im = true;
}

void ignite() {
  // If both triggers are active, find the 3d points corresponding to the
  // hsv-filtered image.
  while(im && cb) {
    for(int rows=0; rows<threshold.rows; rows++) {
      for(int cols=0; cols<threshold.cols; cols++) {
        int value = threshold.data[threshold.channels()*(threshold.cols*rows+cols)+0];
        // ONLY keep WHITE regions
        if(value!=255) {
          cloud->at(cols,rows).x = 0;
          cloud->at(cols,rows).y = 0;
          cloud->at(cols,rows).z = 0;
        }
      }
    }
    pub.publish(*cloud);
    im = false;
    cb = false;
  }
}
