#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "types.h"

// Global variables
ros::Publisher pub;
boost::shared_ptr<tf::TransformListener> listener_;
std::string target_frame = "Elem_0";

/**
 * Callback that receives the pointcloud message and transforms it to the world coordinate system
 */
void cloud_cb_ (const PointCloudCTConstPtr& cloud_msg) {

  PointCloudCTPtr transformed_cloud (new PointCloudCT);
  /*
   * The TransformListener argument that is used in pcl_ros::transformPointCloud
   * gets the parent frame_id from the input pointcloud (here: cloud_msg).
   * To make the frame names consistent, frame_id of cloud_msg is renamed to
   * the correct convention
   */
  // make a deep clone of cloud_msg
  boost::shared_ptr<PointCloudCT> source_cloud(
    new PointCloudCT(*cloud_msg));
  // rename the frame_id
  source_cloud->header.frame_id = "camera_rgb_optical_frame";
  pcl_ros::transformPointCloud(target_frame, *cloud_msg, *transformed_cloud, *listener_);
  // publish the transformed pointcloud
  pub.publish(transformed_cloud);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_tf_listener");
  std::cout << "node: cam_tf_listener" << std::endl;
  ros::NodeHandle node;

  // reading subscriber topic from cmd (if available)
  std::string in_cloud_topic;
  if(argc > 1)
    in_cloud_topic = argv[1];
  else
    in_cloud_topic = "/crops/vision/pointcloud_color_filtered";

  std::cout << "subscribing to topic: " << in_cloud_topic << std::endl;
  listener_.reset(new tf::TransformListener);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = node.subscribe(in_cloud_topic, 1, cloud_cb_);
  // Create a ROS publisher for the output point cloud
  pub = node.advertise<PointCloudCT>("crops/vision/pointcloud_wcs", 1);
  // Spin
  ros::spin();

  return 0;
}
