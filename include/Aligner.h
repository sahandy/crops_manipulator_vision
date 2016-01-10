#ifndef CROPS_VISION_ALIGNER_H__
#define CROPS_VISION_ALIGNER_H__

#include <string>

#include <Eigen/Core>

#include <std_msgs/Float32MultiArray.h>

#include "types.h"

struct AlignParams {
  AlignParams()
    : max_iter(200), num_samples(3), corr_randomness(5),
      sim_thresh(0.5f), max_corr_dist(2.5f), inlier_fraction(0.25f) {}

  int max_iter;
  int num_samples;
  int corr_randomness;
  float sim_thresh;
  float max_corr_dist;
  float inlier_fraction;
};

class Aligner {
public:
  Aligner(std::string model_path);
  bool align(const PointCloudTConstPtr&);
  float* getFruitCenter();
  void getAlignedModel(PointCloudNT::Ptr&);
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
  Eigen::Matrix3f getFinalRotation();
private:
  void load_model(std::string file_path);
  AlignParams align_params_;
  Eigen::Matrix4f transformation_;
  PointCloudNT::Ptr model_;
  FeatureCloudT::Ptr model_features_;
  FeatureCloudT::Ptr target_features_;
  PointCloudNT::Ptr model_aligned_;
  const float leaf_size_;
  std::string model_file_path_;
  float fruit_center_[3];
};


#endif // CROPS_VISION_ALIGNER_H__
