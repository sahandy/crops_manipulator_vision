#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>

#include "types.h"

/**
 * The node is responsible for trimming the point cloud to the size of the
 * workspace. The workspace is trimmed to a default value until the stem is
 * detected. It is then trimmed based on the location of the stem.
 */

// Global variables and constants
ros::Publisher pub_;
ros::Publisher req_stem_pub_;
ros::Subscriber stem_pos_sub_;
ros::Subscriber cloud_sub_;
ros::Subscriber reset_box_sub_;

Eigen::Vector4f minPoint_, maxPoint_;

// default value for stem radius
const float stem_radius_ = 0.20f;

// forward decleration
/**
 * Callback that receives a pointcloud from the given ros topic and performs
 * the trimming. Uses PCL's CropBox to remove outliers.
 */
void cloud_cb_(const PointCloudCTConstPtr& cloud_msg);
/**
 * Callback that receives the X and Y position of the stem
 */
void stem_cb_(const std_msgs::Float32MultiArray::ConstPtr& msg);
/**
 * Callback to reset the crop box values to default
 */
void reset_box_cb_(const std_msgs::Empty::ConstPtr& msg);
/**
 * loads the default values for the crop box filter
 */
void loadDefaultBoxValues();

// Main
int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "crop_workspace");
  ros::NodeHandle nh;
  // reading subscriber topic from cmd (if available)
  std::string in_cloud_topic;
  // init: box default values
  loadDefaultBoxValues();

  if(argc > 1)
    in_cloud_topic = argv[1];
  else
    in_cloud_topic = "crops/vision/pointcloud_wcs"; // default

  cloud_sub_ = nh.subscribe(in_cloud_topic, 1, cloud_cb_);
  stem_pos_sub_ = nh.subscribe("/crops/vision/stem_detector/stem_loc", 1, stem_cb_);
  reset_box_sub_ = nh.subscribe("/crops/vision/crop_box/reset", 1, reset_box_cb_);

  pub_ = nh.advertise<PointCloudCT> ("crops/vision/pointcloud_workspace", 1);

  // Spin
  ros::spin ();

  return 0;
}

void cloud_cb_ (const PointCloudCTConstPtr& cloud_msg) {
  PointCloudCTPtr cropped_cloud (new PointCloudCT);
  // crop all the data inside the given boax
  pcl::CropBox<PointCT> cropFilter;
  cropFilter.setInputCloud(cloud_msg);
  cropFilter.setMin(minPoint_);
  cropFilter.setMax(maxPoint_);
  cropFilter.filter(*cropped_cloud);

  // publish the cropped pointcloud
  pub_.publish (*cropped_cloud);
}

void stem_cb_(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  std::cout << "StemLocator: Got the stem position" << std::endl;
  float *box_center = new float[2];
  box_center[0] = msg->data[0];
  box_center[1] = msg->data[1];

  // create a box around the stem
  minPoint_[0] = box_center[0] - stem_radius_;
  minPoint_[1] = box_center[1] - stem_radius_;
  minPoint_[2] = 0.0;

  maxPoint_[0] = box_center[0] + stem_radius_;
  maxPoint_[1] = box_center[1] + stem_radius_;
  maxPoint_[2] = 1.6;
}

void loadDefaultBoxValues() {
  minPoint_[0] = -1.0; minPoint_[1] = 0.0; minPoint_[2] = 0.0;
  maxPoint_[0] = 1.0; maxPoint_[1] = 1.5; maxPoint_[2] = 1.6;
  std::cout << "box values reset to default." << std::endl;
}

void reset_box_cb_(const std_msgs::Empty::ConstPtr& msg) {
  std::cout << "callback reset_box_cb_" << std::endl;
  loadDefaultBoxValues();
}
