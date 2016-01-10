#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "ClusterExtractor.h"
#include "Aligner.h"
#include "types.h"

typedef int fruit_id;

class Fruit {
public:
  Fruit() : id_(0) {
    PointT p(0.0f, 0.0f, 0.0f);
    center_ = p;
  }

  Fruit(float* p) : id_(0) {
    center_.x = p[0];
    center_.y = p[2];
    center_.z = p[1];
  }

  virtual ~Fruit() {}
  fruit_id id() { return id_; }
  PointT center() { return center_; }
  void set_id(fruit_id id) { id_ = id; }
  void set_center(PointT center) { center_ = center; }
private:
  fruit_id id_;
  PointT center_;
};

class FruitDetector {
public:
  FruitDetector(std::string model_path)
    : in_cloud_topic_("crops/vision/pointcloud_workspace"),
      cluster_extractor_(new ClusterExtractor()),
      aligner_(new Aligner(model_path)) {
    nh_.reset(new ros::NodeHandle);
    registerSubsribers();
    registerPublishers();
  }

private:
  void state_cb_(const std_msgs::Bool::ConstPtr& state_msg);
  void cloud_cb_(const PointCloudTConstPtr& cloud_msg);
  void align_params_cb_(const std_msgs::Float32MultiArray::ConstPtr& params);
  void registerSubsribers();
  void registerPublishers();
  /**
   * Checks if the rotation of aligned fruit around X and Y axes are small and
   * reject those with large values.
   * Normally a fruit hanging from a stem should be upright. Thus, it cannot
   * have a large rotation around X and Y axes.
   */
  bool earlyRejectAlignment(const Eigen::Matrix3f& rotation);
  /**
   * Searches for any similar fruit that has already been detected by the
   * algorithm, based on their center point. If the target center point is
   * close 'enough' to any previsouly deteted fruit, its corresponding fruit_id
   * is returned.
   */
  fruit_id searchFruit(float* center);
  boost::shared_ptr<ros::NodeHandle> nh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber align_state_sub_;
  ros::Subscriber align_params_sub_;
  std::string in_cloud_topic_;
  ros::Publisher fruit_center_pub_;
  ros::Publisher aligned_model_pub_;
  boost::shared_ptr<Aligner> aligner_;
  boost::shared_ptr<ClusterExtractor> cluster_extractor_;
  std::vector<boost::shared_ptr<Fruit> > fruits_;
};

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
  // 10 degrees = 0.17 radians
  if(roll < 0.17 && pitch < 0.17)
    return false;
  return true;
}

fruit_id FruitDetector::searchFruit(float* center) {
  for(int i=0; i<fruits_.size(); ++i) {
    PointT p(center[0], center[1], center[2]);
    float dist = pcl::euclideanDistance(p, fruits_[i]->center());
    if (dist < 0.05f)
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

  // early rejection?
  // can be done using clusterizer parameters --> setMinClusterSize

  // alignment (for each cluster)
  size_t sz = clusters.size();
  std::cout << "found #" << sz << " clusters" << std::endl;
  for(size_t i=0; i<sz; ++i) {
    std::cout << "showing cluster #" << i << std::endl;
    aligned_model_pub_.publish(*clusters[i]);
    ros::Duration(2.0).sleep();
  }

  for(size_t i=0; i<sz; ++i) {
    if( aligner_->align(clusters[i]) ) {
        Eigen::Matrix3f rot = aligner_->getFinalRotation();
      if(earlyRejectAlignment(rot))
        continue;
      float *center = new float[3];
      center = aligner_->getFruitCenter();
      fruit_id foundId = searchFruit(center);
      if(foundId == -1) {
        fruits_.push_back(boost::shared_ptr<Fruit>(new Fruit(center)));
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
    }
  }
}


int main(int argc, char** argv) {
  ros::init (argc, argv, "detector");

  // load the sweet paprika model
  // struct that helps finding the current home directory
  struct passwd *pwd;
  int id = 0;
  if( (pwd = getpwuid(getuid())) == NULL ) {
    ROS_FATAL("Error finding the home directory.");
    return 0;
  }
  std::stringstream ss;
  ss << pwd->pw_dir << "/.crops_manipulator/vision/model/";
  std::cout << "Model located at: " << ss.str() << std::endl;

  std::string full_path;
  std::string file_name = "paprika_centered.pcd";
  full_path.append(ss.str());
  full_path.append(file_name);

  FruitDetector fd(full_path);

  ros::spin();

  return 0;
}
