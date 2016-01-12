#ifndef CROPS_VISION_FRUIT_DETECTOR_H__
#define CROPS_VISION_FRUIT_DETECTOR_H__

#include <string>
#include <vector>

#include <pcl/point_cloud.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>

#include "ClusterExtractor.h"
#include "Aligner.h"
#include "types.h"

struct RPY {
  float roll;
  float pitch;
  float yaw;
};

// typedef for convenience when dealing with different fruits
typedef int fruit_id;

// Convenient class to define fruit types
class Fruit {
public:
  Fruit() : id_(0) {
    PointT p(0.0f, 0.0f, 0.0f);
    center_ = p;
  }
  /**
   * Constructor with specified fruit center point
   */
  Fruit(float* p, Eigen::Matrix3f r) : id_(0), rotation_(r) {
    center_.x = p[0];
    center_.y = p[1];
    center_.z = p[2];
  }

  virtual ~Fruit() {}
  // Accessors
  fruit_id id() { return id_; }
  PointT center() { return center_; }
  Eigen::Matrix3f rotation() { return rotation_; }
  float roll() { return rpy_.roll; }
  float pitch() { return rpy_.pitch; }
  float yaw() { return rpy_.yaw; }
  RPY get_RPY() { return rpy_; }
  // Setters
  void set_id(fruit_id id) { id_ = id; }
  void set_center(PointT center) { center_ = center; }
  void set_rotation(Eigen::Matrix3f r) {
    rotation_ = r;
    const Eigen::Vector3f euler_angles = rotation_.eulerAngles(2, 1, 0);
    // also set the RPY based on the rotation matrix
    rpy_.yaw = euler_angles[0];
    rpy_.pitch = euler_angles[1];
    rpy_.roll = euler_angles[2];
  }

  void setRPY(float r, float p, float y) {
    rpy_.roll = r;
    rpy_.pitch = p;
    rpy_.yaw = y;
  }

private:
  /**
   * Instance that holds a unique identifier for the fruit
   */
  fruit_id id_;
  /**
   * PCL Point type instance that holds the center point of the fruit
   */
  PointT center_;
  Eigen::Matrix3f rotation_;
  RPY rpy_;
};

class FruitDetector {
public:
  FruitDetector(std::string model_path);
  virtual ~FruitDetector() {}
private:
  /**
   * Callback that receives a boolean message to toggle the state of the
   * detection to ACTIVE or INACTIVE
   */
  void state_cb_(const std_msgs::Bool::ConstPtr& state_msg);
  /**
   * Callback that receives a pointcloud on a given topic and triggers the
   * detection procedure.
   */
  void cloud_cb_(const PointCloudTConstPtr& cloud_msg);
  /**
   * Callback that receives a message contaning an array of parameters to
   * modify the alignment parameters.
   */
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
  bool enhanceAlignment(
    const Eigen::Matrix3f& new_rotation, const boost::shared_ptr<Fruit>& f);
  /**
   * Searches for any similar fruit that has already been detected by the
   * algorithm, based on their center point. If the target center point is
   * close 'enough' to any previsouly deteted fruit, its corresponding fruit_id
   * is returned.
   */
  fruit_id searchFruit(float* center);
  /**
   * ROS Node Handle for this node.
   */
  boost::shared_ptr<ros::NodeHandle> nh_;
  /**
   * ROS Subscriber to toggle the state of detection
   */
  ros::Subscriber align_state_sub_;
  /**
   * ROS Subscriber to the pointcloud of a specific topic
   */
  ros::Subscriber cloud_sub_;
  /**
   * ROS Subscriber to receive the alignment parameters from the
   * GUI::VisionConfig
   */
  ros::Subscriber align_params_sub_;
  /**
   * String instance that holds the topic of the incoming point cloud.
   */
  std::string in_cloud_topic_;
  /**
   * ROS Publisher to advertise center point of a newly detected fruit.
   */
  ros::Publisher fruit_center_pub_;
  /**
   * ROS Publisher to advertise the aligned fruit model.
   */
  ros::Publisher aligned_model_pub_;
  /**
   * Instance of Aligner class
   */
  boost::shared_ptr<Aligner> aligner_;
  /**
   * Instance of ClusterExtractor class
   */
  boost::shared_ptr<ClusterExtractor> cluster_extractor_;
  /**
   * Vector containing the detected fruits in the scene.
   */
  std::vector<boost::shared_ptr<Fruit> > fruits_;
};

#endif // CROPS_VISION_FRUIT_DETECTOR_H__
