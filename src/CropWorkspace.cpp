#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>

// Helper typedefs to make the implementation code cleaner
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef typename PointCloudT::Ptr PointCloudPtr;
typedef typename PointCloudT::ConstPtr PointCloudConstPtr;

// Global variables and constants
ros::Publisher pub;

// forward decleration
void cloud_cb_ (const PointCloudConstPtr& cloud_msg);

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "crop_workspace");
  ros::NodeHandle nh;
  // reading subscriber topic from cmd (if available)
  std::string in_cloud_topic;
  if(argc > 1)
    in_cloud_topic = argv[1];
  else
    in_cloud_topic = "crops/vision/pointcloud_wcs"; // default
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (in_cloud_topic, 1, cloud_cb_);
  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<PointCloudT> ("crops/vision/pointcloud_workspace", 1);
  // Spin
  ros::spin ();

  return 0;
}

/**
 * Callback method to receive a pointcloud from the given ros topic
 */
void cloud_cb_ (const PointCloudConstPtr& cloud_msg) {
  PointCloudPtr cropped_cloud (new PointCloudT);
  Eigen::Vector4f minPoint, maxPoint;
  minPoint[0] = -1.0; minPoint[1] = 0.0; minPoint[2] = 0.0;
  maxPoint[0] = 1.0; maxPoint[1] = 1.5; maxPoint[2] = 1.5;
  // crop all the data inside the given boax
  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud(cloud_msg);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.filter(*cropped_cloud);

  // publish the cropped pointcloud
  pub.publish (*cropped_cloud);
}
