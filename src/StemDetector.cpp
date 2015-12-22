#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <string>
#include <iostream>
#include <vector>
#include <limits>

#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

#include <Eigen/Core>

#include "types.h"

/*
 * Global variables
 */
boost::shared_ptr<ros::NodeHandle> nh_;
ros::Subscriber cloud_sub_;
ros::Subscriber worker_sub_;
ros::Publisher stem_model_pub_;
ros::Publisher stem_x_pub_;

std::string in_cloud_topic_;
double cylinder_radius_;
int num_observations_;
std::vector<float> stem_observations_;
PointCloudTPtr stem_model_;
bool stem_model_publisher_lock;
// instance that holds the calculated X value for the stem
std_msgs::Float32 msg_stem_x_;

/*
 * functions
 */
void get_stem_x_cb_(const std_msgs::Float32MultiArray::ConstPtr& msg);
void subscribe();
void unsubsribe();
std_msgs::Float32 getOptimalStemX();
void publishStemModel();
void cloud_cb_(const PointCloudTConstPtr& cloud_msg);

/**
 * MAIN
 */
int main(int argc, char** argv) {
  ros::init (argc, argv, "stem_detector");
  nh_.reset(new ros::NodeHandle);
  // reading subscriber topic from cmd (if available)
  if(argc > 1)
    in_cloud_topic_ = argv[1];
  else
    in_cloud_topic_ = "crops/vision/pointcloud_workspace";
  // set default values
  num_observations_ = 50;
  cylinder_radius_ = 0.04;
  stem_model_publisher_lock = true;

  worker_sub_ = nh_->subscribe("/crops/vision/stem_detector/get_stem_position", 1, get_stem_x_cb_);
  stem_model_pub_ = nh_->advertise<PointCloudT> ("/crops/vision/stem_detector/stem_model", 1);
  stem_x_pub_ = nh_->advertise<std_msgs::Float32> ("/crops/vision/stem_detector/stem_x", 1);

  ros::Rate rate(30); // 30Hz
  while(ros::ok()) {
    if(!stem_model_publisher_lock)
      publishStemModel();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

/**
 * Callback that incorporates the incoming variables from the GUI and computes
 * the X position for the stem.
 * Values included in the incoming message:
 *        0. whether to return the old value or compute a new one
 *              [0] return old X value
 *              [1] compute a new X value
 *        1. number of observations
 *        2. cylinder model's radius limit
 */
void get_stem_x_cb_(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if( msg->data[0] == 0 )
    stem_x_pub_.publish(msg_stem_x_);
  else {
    num_observations_ = (int) msg->data[1];
    cylinder_radius_ = msg->data[0];
    stem_model_publisher_lock = true;
    std::cout << "computing stem position..." << std::endl;
    subscribe();
  }
}

/**
 * Subscribe to the Pointcloud topic
 */
void subscribe() {
  cloud_sub_ = nh_->subscribe(in_cloud_topic_, 1, cloud_cb_);
}

/**
 * Unsubsribe from the Pointcloud topic
 */
void unsubsribe() {
  cloud_sub_.shutdown();
}

void publishStemModel() {
  stem_model_pub_.publish(stem_model_);
}

std_msgs::Float32 getOptimalStemX() {
  std_msgs::Float32 msg;
  double mean, std_dev;
  pcl::getMeanStd(stem_observations_, mean, std_dev);
  int sz = stem_observations_.size();
  std::cout << "out of " << sz << " observations...\nMean: " << mean
            << "\tStd_Dev: " << std_dev << std::endl;
  msg.data = mean;

  return msg;
}

void cloud_cb_(const PointCloudTConstPtr& cloud_msg) {
  PointCloudNTPtr cloud_msg_normals(new PointCloudNT);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

  // estimating cloud_msg normals
  pcl::NormalEstimationOMP<PointT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (cloud_msg);
  nest.compute (*cloud_msg_normals);

  pcl::SACSegmentationFromNormals<PointT, PointNT> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1); // surface normal influence
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, cylinder_radius_); // limit the radius of cylindrical model to be smaller than 4cm
  seg.setInputCloud (cloud_msg);
  seg.setInputNormals (cloud_msg_normals);
  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud_msg);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  PointCloudTPtr cloud_cylinder(new PointCloudT);
  extract.filter (*cloud_cylinder);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_cylinder, centroid);
  
  const float centroid_x = centroid(0);
  // FIXME put correct limits for the if-statement
  if(0.1 < centroid_x && centroid_x < 1.0)
    stem_observations_.push_back(centroid_x);
  if(stem_observations_.size() >= num_observations_) {
    unsubsribe();
    // compute the optimal stem position (x value)
    msg_stem_x_ = getOptimalStemX();
    stem_x_pub_.publish(msg_stem_x_);
    stem_model_.reset(new PointCloudT);
    pcl::copyPointCloud(*cloud_cylinder, *stem_model_);
    // clear the observations container
    stem_observations_.clear();
    // Unlock the model publisher
    stem_model_publisher_lock = false;
  }
}
