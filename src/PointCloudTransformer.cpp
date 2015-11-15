#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>


// Global variables
ros::Publisher pub;
boost::shared_ptr<tf::TransformListener> listener_;

/**
 * Callback that receives the pointcloud message and transforms it to the world coordinate system
 */
void cloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  boost::shared_ptr<sensor_msgs::PointCloud2> source_cloud;
  sensor_msgs::PointCloud2Ptr transformed_cloud;
  /*
   * The TransformListener argument that is used in pcl_ros::transformPointCloud gets the parent frame_id from the
   * input pointcloud (here: cloud_msg).
   * To make the frame names consistent, frame_id of cloud_msg is renamed to the correct convention
   */
  // make a deep clone of cloud_msg
  source_cloud = boost::make_shared<sensor_msgs::PointCloud2>(*cloud_msg);
  // rename the frame_id
  source_cloud->header.frame_id = "camera";
  std::string target_frame = "Elem_0";
  pcl_ros::transformPointCloud(target_frame, *source_cloud, *transformed_cloud, *listener_);

  // publish the transformed pointcloud
  pub.publish(transformed_cloud);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_tf_listener");

  ros::NodeHandle node;

  listener_.reset(new tf::TransformListener);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = node.subscribe ("/camera/depth_registered/points", 1, cloud_cb_);
  // Create a ROS publisher for the output point cloud
  pub = node.advertise<sensor_msgs::PointCloud2> ("transformed_cloud", 1);
  // Spin
  ros::spin();

  return 0;
}
