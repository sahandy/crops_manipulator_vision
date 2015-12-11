#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>

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

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "types.h"

ros::Publisher pub;
ros::Publisher model_pub;
ros::Publisher fruit_center_pub;
ros::Subscriber sub;

PointCloudNT::Ptr model (new PointCloudNT);
PointCloudNT::Ptr model_aligned (new PointCloudNT);
FeatureCloudT::Ptr model_features (new FeatureCloudT);
FeatureCloudT::Ptr target_features (new FeatureCloudT);

const float leaf_size = 0.005f;

// forward decleration
void load_model(std::string file_path);
void cloud_cb_ (const PointCloudTConstPtr& cloud_msg);

int main(int argc, char** argv) {
  ros::init (argc, argv, "align_fruit");
  ros::NodeHandle nh_;
  // reading subscriber topic from cmd (if available)
  std::string in_cloud_topic;
  if(argc > 1)
    in_cloud_topic = argv[1];
  else
    in_cloud_topic = "crops/vision/pointcloud_workspace";

  std::cout << "subscribing to topic: " << in_cloud_topic << std::endl;
  ros::Subscriber sub =
    nh_.subscribe (in_cloud_topic, 1, cloud_cb_);
  pub = nh_.advertise<PointCloudT> ("/crops/vision/aligned_cloud", 1);
  model_pub = nh_.advertise<PointCloudNT> ("/crops/vision/model", 1);
  fruit_center_pub =
    nh_.advertise<geometry_msgs::Vector3> ("/crops/vision/result/fruit_center", 1);
  // load the sweet paprika model
  // struct that helps finding the current home directory
  struct passwd *pwd;
  int id = 0;
  if( (pwd = getpwuid(getuid())) == NULL ) {
    ROS_FATAL("Error finding the home directory.");
    return 0;
  }
  std::stringstream ss;
  ss << pwd->pw_dir << "/.crops_manipulator/vision/model/";
  std::cout << "Model located at: " << ss.str() << std::endl;

  std::string full_path;
  std::string file_name = "paprika_centered.pcd";
  full_path.append(ss.str());
  full_path.append(file_name);

  load_model(full_path);

  // while(true) {
  //   pub.publish(*cloud);
  //   ros::spinOnce();
  // }

  ros::spin();
  return 0;
}

void load_model(std::string file_path) {
  PointCloudTPtr cloud (new PointCloudT);
  PointCloudTPtr cloud_filtered (new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1) {
    PCL_ERROR("Could not find the file\n");
  }
  // voxelize model
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*cloud_filtered);
  cloud_filtered->header.frame_id = "/Elem_0";
  // copy to the point cloud of type 'PointNormal'
  pcl::copyPointCloud(*cloud_filtered, *model);

  std::cout << "Model size: " << model->points.size() << std::endl;
}
void cloud_cb_ (const PointCloudTConstPtr& cloud_msg) {
  // DEBUG: Publish model
  pub.publish(*model);

  PointCloudTPtr cloud_filtered (new PointCloudT);
  // voxelize model
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud_msg);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
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
  fest.setInputCloud (model);
  fest.setInputNormals (model);
  fest.compute (*model_features);
  fest.setInputCloud (target);
  fest.setInputNormals (target);
  fest.compute (*target_features);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (model);
  align.setSourceFeatures (model_features);
  align.setInputTarget (target);
  align.setTargetFeatures (target_features);
  align.setMaximumIterations (200); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.5f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf_size); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*model_aligned);
  }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), model->size ());

    geometry_msgs::Vector3 msg;
    float x, y, z, temp;
    temp = transformation(0, 3) * 100.0f;
    x = ((int)temp) / 100.0f;
    temp = transformation(1, 3) * 100.0f;
    y = ((int)temp) / 100.0f;
    temp = transformation(2, 3) * 100.0f;
    z = ((int)temp) / 100.0f;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    fruit_center_pub.publish(msg);
    // Show alignment
    PointCloudTPtr output (new PointCloudT);
    pcl::transformPointCloud(*model, *model, transformation);
    pcl::copyPointCloud(*model, *output);
    pub.publish(*output);
    // ros::Duration(1.0).sleep();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    std::cout << "waiting for the next cloud..." << std::endl;
    // return (1);
  }

}
