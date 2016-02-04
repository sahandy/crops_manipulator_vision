#include "ClusterExtractor.h"

#include <iostream>

ClusterExtractor::ClusterExtractor()
    : kd_tree_(new pcl::search::KdTree<PointT>()) {
    clusterizer_.setClusterTolerance(0.02); // 2cm
    clusterizer_.setMinClusterSize(150);
    clusterizer_.setMaxClusterSize(1900); // paprika half model: 3835/2 points
    clusterizer_.setSearchMethod(kd_tree_);
}

void ClusterExtractor::setInputCloud(const PointCloudTConstPtr cloud) {
  input_cloud_.reset(new PointCloudT);
  pcl::copyPointCloud(*cloud, *input_cloud_);
  kd_tree_->setInputCloud(input_cloud_);
  clusterizer_.setSearchMethod(kd_tree_);
  clusterizer_.setInputCloud(input_cloud_);
}

void ClusterExtractor::extract(std::vector<PointCloudTPtr> &cloud_clusters) {
  std::vector<pcl::PointIndices> cluster_indices;
  clusterizer_.extract(cluster_indices);
  // read cluster_indices and create a point cloud out of each set
  for(
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    it != cluster_indices.end (); ++it) {
    PointCloudTPtr single_cluster(new PointCloudT);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      single_cluster->points.push_back (input_cloud_->points[*pit]);
    // Prepare the rest of point cloud attributes before saving into vector
    single_cluster->width = single_cluster->points.size();
    single_cluster->height = 1;
    // set the frame name to be recognized by RVIZ and other TF Listeners
    single_cluster->header.frame_id = "/Elem_0";

    cloud_clusters.push_back(single_cluster);
    }
}
