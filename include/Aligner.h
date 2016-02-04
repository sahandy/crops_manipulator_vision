#ifndef CROPS_VISION_ALIGNER_H__
#define CROPS_VISION_ALIGNER_H__

#include <string>

#include <Eigen/Core>

#include <std_msgs/Float32MultiArray.h>

#include "types.h"

/**
 * Struct that holds the alignment parameters needed by the RANSAC algorithm.
 */
struct AlignParams {
  AlignParams()
    : max_iter(200), num_samples(3), corr_randomness(5),
      sim_thresh(0.5f), max_corr_dist(2.5f), inlier_fraction(0.25f) {}
  // maximum number of iterations allowed for each time algorithm runs
  int max_iter;
  // Number of points to sample for generating/prerejecting a pose
  int num_samples;
  // Number of nearest features to use
  int corr_randomness;
  // Polygonal edge length similarity threshold
  float sim_thresh;
  // Inlier threshold
  float max_corr_dist;
  // Required inlier fraction for accepting a pose hypothesis
  float inlier_fraction;
};

/**
 * Instance that is used by the FruitDetector to align a reference model to a
 * given point cloud. The class loads a model from disk and receives a point
 * cloud as input. It then tries to align the model to the input. If successfull,
 * the transformation matrix is stored.
 */
class Aligner {
public:
  Aligner(std::string model_path);
  /**
   * Function that performs
   *  1. feature extraction (model, target),
   *  2. alignment
   */
  bool align(const PointCloudTConstPtr&);
  /**
   * Accessor for any other class to get the detected fruit's center point
   */
  float* getFruitCenter();
  /**
   * Accessor for any other class to get the transformed model to the aligned
   * position.
   */
  void getAlignedModel(PointCloudNT::Ptr&);
  /**
   * Accessor for any other class to get the rotation result of the alignment.
   */
  Eigen::Matrix3f getFinalRotation();
  /**
   * Callback that receives the message containing new alignment parameters.
   * CAUTION: order of attributes in the message
   *        0. Maximum Iterations
   *        1. Number of Samples
   *        2. Correspondence Randomness
   *        3. Similarity Threshold
   *        4. Max Correspondence Distance
   *        5. Inlier Fraction
   */
  void align_params_cb_(const std_msgs::Float32MultiArray::ConstPtr& params);
private:
  void load_model(std::string file_path);
  AlignParams align_params_;
  Eigen::Matrix4f transformation_;
  // reference model
  PointCloudNT::Ptr model_;
  // features extracted from the reference model
  FeatureCloudT::Ptr model_features_;
  // features extracted from the input point cloud
  FeatureCloudT::Ptr target_features_;
  // point cloud instance that is transformed to the position of the alignment
  PointCloudNT::Ptr model_aligned_;
  // point cloud sampling size (voxel size)
  const float leaf_size_;
  // string that holds the path to the location of the model's file on disk
  std::string model_file_path_;
  // array to hold [x, y, z] values of the detected fruit's center point
  float fruit_center_[3];
};


#endif // CROPS_VISION_ALIGNER_H__
