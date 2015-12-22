#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <string>
#include <iostream>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "types.h"

// Global variables
boost::shared_ptr<ros::NodeHandle> nh_;
ros::Subscriber cloud_sub_;
ros::Publisher cluster_pub_;

std::string in_cloud_topic_;

void cloud_cb_(const PointCloudTConstPtr& cloud_msg);

int main(int argc, char** argv) {
  ros::init (argc, argv, "clusterizer");
  nh_.reset(new ros::NodeHandle);
  // reading subscriber topic from cmd (if available)
  if(argc > 1)
    in_cloud_topic_ = argv[1];
  else
    in_cloud_topic_ = "crops/vision/pointcloud_workspace";

  cloud_sub_ = nh_->subscribe(in_cloud_topic_, 1, cloud_cb_);
  cluster_pub_ = nh_->advertise<PointCloudT> ("/crops/vision/cloud_clusters", 1);

  ros::spin();

  return 0;
}

void cloud_cb_(const PointCloudTConstPtr& cloud_msg) {
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_msg);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> clusterizer_;
  clusterizer_.setClusterTolerance(0.02); // 2cm
  clusterizer_.setMinClusterSize(100);
  clusterizer_.setMaxClusterSize(3500); // paprika model: 3800 points
  clusterizer_.setSearchMethod(tree);
  clusterizer_.setInputCloud(cloud_msg);
  clusterizer_.extract(cluster_indices);

  std::vector<PointCloudTPtr> cloud_clusters;
  for(
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    it != cluster_indices.end (); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr single_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      single_cluster->points.push_back (cloud_msg->points[*pit]);
    single_cluster->width = single_cluster->points.size();
    single_cluster->height = 1;
    single_cluster->header.frame_id = "/Elem_0";
  
    cloud_clusters.push_back(single_cluster);
    }

  int sz = cloud_clusters.size();
  std::cout << "#clusters: " << sz << std::endl;
  for(int i=0; i<sz; ++i) {
    std::cout << "publishing cluster #" << i << std::endl;
    cluster_pub_.publish(*cloud_clusters[i]);
    ros::Duration(1.0).sleep();
  }
}
