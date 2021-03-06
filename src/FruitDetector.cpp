#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Geometry>

#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>

#include "FruitDetector.h"

FruitDetector::FruitDetector(std::string model_path)
  : in_cloud_topic_("crops/vision/pointcloud_workspace"),
    cluster_extractor_(new ClusterExtractor()),
    aligner_(new Aligner(model_path)) {
  nh_.reset(new ros::NodeHandle);
  registerSubsribers();
  registerPublishers();
}

void FruitDetector::registerSubsribers() {
  align_state_sub_ = nh_->subscribe("/crops/vision/alignment/state", 1,
    &FruitDetector::state_cb_, this);
  align_params_sub_ = nh_->subscribe("/crops/vision/alignment/params", 1,
    &Aligner::align_params_cb_, &*aligner_);
}

void FruitDetector::registerPublishers() {
  fruit_center_pub_ =
    nh_->advertise<geometry_msgs::Vector3> ("/crops/vision/result/fruit_center", 1);
  aligned_model_pub_ = nh_->advertise<PointCloudT> ("/crops/vision/aligned_cloud", 1);
}

void FruitDetector::state_cb_(const std_msgs::Bool::ConstPtr& state_msg) {
  const bool active = state_msg->data;
  if(active)
    cloud_sub_ = nh_->subscribe(in_cloud_topic_, 1,
      &FruitDetector::cloud_cb_, this);
  else
    cloud_sub_.shutdown();
}

bool FruitDetector::earlyRejectAlignment(const Eigen::Matrix3f& rotation) {
  // get euler angles. (Z-Y-X) convention --> (Yaw, Pitch, Roll)
  const Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0);
  const float yaw = euler_angles[0];
  const float pitch = euler_angles[1];
  const float roll = euler_angles[2];
  std::cout << "roll: " << roll << "\npitch: " << pitch << "\nyaw: " << yaw << std::endl;
  // 20 degrees = 0.35 radians
  if(roll < 0.35 && pitch < 0.35)
    return false;
  return true;
}

bool FruitDetector::enhanceAlignment(
  const Eigen::Matrix3f& new_rotation, const boost::shared_ptr<Fruit>& f) {
  // get euler angles. (Z-Y-X) convention --> (Yaw, Pitch, Roll)
  const Eigen::Vector3f euler_angles = new_rotation.eulerAngles(2, 1, 0);
  const float new_yaw = euler_angles[0];
  const float new_pitch = euler_angles[1];
  const float new_roll = euler_angles[2];

  const float old_roll = f->get_RPY().roll;
  const float old_pitch = f->get_RPY().pitch;
  // check if the new aligned model is less tilted
  if (new_roll < old_roll && new_pitch < old_pitch)
    return true;
  return false;
}

fruit_id FruitDetector::searchFruit(float* center) {
  printf("searchFruit(%f, %f, %f)\n", center[0], center[1], center[2]);
  std::cout << "fruits_.size(): " << fruits_.size() << std::endl;
  for(int i=0; i<fruits_.size(); ++i) {
    PointT p(center[0], center[1], center[2]);
    std::cout << "fruits[" << i << "].center: "
              << fruits_[i]->center().x << ", "
              << fruits_[i]->center().y << ", "
              << fruits_[i]->center().z << std::endl;
    float const dist = pcl::euclideanDistance(p, fruits_[i]->center());
    std::cout << "dist[" << i << "]: " << dist << std::endl;
    if (dist < 0.10f)
      return i;
  }
  // didn't find the fruit, return -1 to indicate this is a new one
  return -1;
}

void FruitDetector::cloud_cb_(const PointCloudTConstPtr& cloud_msg) {
  PointCloudTPtr input_cloud (new PointCloudT);
  // vector containing different clusters extracted from the input_cloud
  std::vector<PointCloudTPtr> clusters;
  pcl::copyPointCloud(*cloud_msg, *input_cloud);
  cluster_extractor_->setInputCloud(input_cloud);
  cluster_extractor_->extract(clusters);

  // clustering: early rejection?
  // already done using clusterizer parameters --> setMinClusterSize

  // alignment (for each cluster)
  size_t sz = clusters.size();
  std::cout << "found #" << sz << " clusters" << std::endl;

  for(size_t i=0; i<sz; ++i) {
    if( aligner_->align(clusters[i]) ) {
        Eigen::Matrix3f rot = aligner_->getFinalRotation();
      if(earlyRejectAlignment(rot))
        continue;
      float *center = new float[3];
      center = aligner_->getFruitCenter();
      fruit_id foundId = searchFruit(center);
      std::cout << "foundId: " << foundId << std::endl;
      if(foundId == -1) {
        // This is a unique new fruit. Add it to the vector
        fruits_.push_back(boost::shared_ptr<Fruit>(new Fruit(center, rot)));
        PointCloudNT::Ptr aligned_model (new PointCloudNT);
        aligner_->getAlignedModel(aligned_model);
        // create messages to pass the result
        geometry_msgs::Vector3 msg_fruit_center;
        msg_fruit_center.x = center[0];
        msg_fruit_center.y = center[1];
        msg_fruit_center.z = center[2];
        // publish the fruit center point data
        fruit_center_pub_.publish(msg_fruit_center);
        // and publish the aligned pointcloud
        aligned_model_pub_.publish(*aligned_model);
        // TODO remove the sleep time
        ros::Duration(2.0).sleep();
      }
      else {
        // the approximation is close to one of the previous fruits
        if(!enhanceAlignment(rot, fruits_[i]))
          continue;
      }

    }
  }
}
