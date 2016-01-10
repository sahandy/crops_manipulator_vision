#ifndef CROPS_VISION_CLUSTER_EXTRACTOR_H__
#define CROPS_VISION_CLUSTER_EXTRACTOR_H__

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

class ClusterExtractor {
public:
  ClusterExtractor();
  void setInputCloud(const PointCloudTConstPtr input_cloud);
  void extract(std::vector<PointCloudTPtr> &cloud_clusters);
private:
  pcl::EuclideanClusterExtraction<PointT> clusterizer_;
  pcl::search::KdTree<PointT>::Ptr kd_tree_;
  PointCloudTPtr input_cloud_;
};

#endif // CROPS_VISION_CLUSTER_EXTRACTOR_H__
