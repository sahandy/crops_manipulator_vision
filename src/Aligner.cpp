#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "Aligner.h"

Aligner::Aligner(std::string model_path)
  : model_file_path_(model_path),
    transformation_(Eigen::Matrix4f::Identity()),
    leaf_size_(0.005f),
    model_(new PointCloudNT),
    model_features_(new FeatureCloudT),
    target_features_(new FeatureCloudT) {
  load_model(model_file_path_);
}

void Aligner::align_params_cb_(
  const std_msgs::Float32MultiArray::ConstPtr& params) {
    std::cout << "Aligner: GOT NEW PARAMETERS" << std::endl;

    align_params_.max_iter        = (int) params->data[0];
    align_params_.num_samples     = (int) params->data[1];
    align_params_.corr_randomness = (int) params->data[2];
    align_params_.sim_thresh      = (float) params->data[3];
    align_params_.max_corr_dist   = (float) params->data[4];
    align_params_.inlier_fraction = (float) params->data[5];
}

void Aligner::load_model(std::string file_path) {
  PointCloudTPtr cloud (new PointCloudT);
  PointCloudTPtr cloud_filtered (new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1) {
    PCL_ERROR("Could not find the file\n");
  }
  // voxelize model
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  vg.filter(*cloud_filtered);
  cloud_filtered->header.frame_id = "/Elem_0";
  // copy to the point cloud of type 'PointNormal'
  pcl::copyPointCloud(*cloud_filtered, *model_);

  std::cout << "Model size: " << model_->points.size() << std::endl;
}

float* Aligner::getFruitCenter() {
  return fruit_center_;
}

void Aligner::getAlignedModel(PointCloudNT::Ptr &result) {
  pcl::copyPointCloud(*model_aligned_, *result);
}

Eigen::Matrix3f Aligner::getFinalRotation() {
  Eigen::Matrix3f rot;
  rot <<  transformation_(0,0), transformation_(0,1), transformation_(0,2),
          transformation_(1,0), transformation_(1,1), transformation_(1,2),
          transformation_(2,0), transformation_(2,1), transformation_(2,2);
  return rot;
}

bool Aligner::align(const PointCloudTConstPtr& cloud_msg) {
  PointCloudTPtr cloud_filtered (new PointCloudT);
  // voxelize model
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud_msg);
  vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  vg.filter(*cloud_filtered);

  PointCloudNT::Ptr target (new PointCloudNT);
  pcl::copyPointCloud(*cloud_filtered, *target);
  // estimating target normals
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (target);
  nest.compute (*target);
  // Estimate features
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (model_);
  fest.setInputNormals (model_);
  fest.compute (*model_features_);
  fest.setInputCloud (target);
  fest.setInputNormals (target);
  fest.compute (*target_features_);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (model_);
  align.setSourceFeatures (model_features_);
  align.setInputTarget (target);
  align.setTargetFeatures (target_features_);
  align.setMaximumIterations (align_params_.max_iter);
  align.setNumberOfSamples (align_params_.num_samples);
  align.setCorrespondenceRandomness (align_params_.corr_randomness);
  align.setSimilarityThreshold (align_params_.sim_thresh);
  align.setMaxCorrespondenceDistance (align_params_.max_corr_dist * leaf_size_);
  align.setInlierFraction (align_params_.inlier_fraction);
  {
    pcl::ScopeTime t("Alignment");
    model_aligned_.reset(new PointCloudNT);
    align.align (*model_aligned_);
  }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    transformation_ = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_ (0,0), transformation_ (0,1), transformation_ (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_ (1,0), transformation_ (1,1), transformation_ (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_ (2,0), transformation_ (2,1), transformation_ (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_ (0,3), transformation_ (1,3), transformation_ (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), model_->size ());

    float x, y, z, temp;
    temp = transformation_(0, 3) * 100.0f;
    x = ((int)temp) / 100.0f;
    temp = transformation_(1, 3) * 100.0f;
    y = ((int)temp) / 100.0f;
    temp = transformation_(2, 3) * 100.0f;
    z = ((int)temp) / 100.0f;

    fruit_center_[0] = x;
    fruit_center_[1] = y;
    fruit_center_[2] = z;

    return true;
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    std::cout << "waiting for the next cloud..." << std::endl;
    return false;
  }

}
