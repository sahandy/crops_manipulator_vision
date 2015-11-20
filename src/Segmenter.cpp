#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>


// Helper typedefs to make the implementation code cleaner
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef typename PointCloudT::Ptr PointCloudPtr;
typedef typename PointCloudT::ConstPtr PointCloudConstPtr;

// Global variables and constants
ros::Publisher pub;
double const min_filter_percentage_ = 0.1;

// forward decleration
void cloud_cb_ (const PointCloudConstPtr& cloud_msg);

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "vision_segmenter");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("crops/vision/pointcloud_workspace", 1, cloud_cb_);
  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<PointCloudT> ("crops/vision/pointcloud_segments", 1);
  // Spin
  ros::spin ();

  return 0;
}

/**
 * Callback method to receive a pointcloud from the given ros topic
 */
void cloud_cb_ (const PointCloudConstPtr& cloud_msg) {
  PointCloudPtr segments (new PointCloudT);
  pcl::copyPointCloud(*cloud_msg, *segments);
  /*
   * Extract Planar surfaces
   */
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr current_plane_indices(new pcl::PointIndices);
  // instance that will be used to extract the points of the largest found
  // plane
  pcl::ExtractIndices<PointT>  extract;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional parameter for optimization
  seg.setOptimizeCoefficients (true);
  // Mandatory parameters required by segmentation algorithm
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold (0.03);

  size_t const original_cloud_size = segments->size();
  size_t const point_threshold = min_filter_percentage_ * original_cloud_size;
  while(segments->size() > point_threshold) {
    seg.setInputCloud (segments);
    // perform segmentation
    seg.segment (*current_plane_indices, coefficients);

    // check if we found any indices in this iteration
    if(current_plane_indices->indices.size() == 0) {
      break;
    }

    // Remove the currently found inliers from the input cloud (prepare the
    // cloud for the next step)
    extract.setInputCloud(segments);
    extract.setIndices(current_plane_indices);
    extract.setNegative(true);
    extract.filter(*segments);
  }

  // publish the result
  pub.publish(*segments);

}
