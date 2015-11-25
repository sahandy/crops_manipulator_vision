#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Helper typedefs to make the implementation code cleaner
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef typename PointCloudT::Ptr PointCloudPtr;
typedef typename PointCloudT::ConstPtr PointCloudConstPtr;

ros::Publisher pub;
image_transport::Subscriber image_sub_;

PointCloudPtr cloud;
cv::Mat threshold;

bool im, cb;

// Forward decleration
void ignite();
void cloud_cb_ (const PointCloudConstPtr& cloud_msg);
void image_cb_ (const sensor_msgs::ImageConstPtr& msg);

int main (int argc, char** argv) {
  im = false;
  cb = false;
  // Initialize ROS
  ros::init (argc, argv, "cloud_color_filter");
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_(nh_);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub =
    nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb_);
  // Create a ROS subscriber for the input color filtered image
  image_sub_ = it_.subscribe("crops/vision/image_filter/hsv_filtered", 1, image_cb_);
  // Create a ROS publisher for the output model coefficients
  pub = nh_.advertise<PointCloudT> ("/crops/vision/pointcloud_color_filtered", 1);

  // Spin
  ros::spin ();

  return 0;
}

void cloud_cb_ (const PointCloudConstPtr& cloud_msg) {
  cloud.reset(new PointCloudT);
  pcl::copyPointCloud(*cloud_msg, *cloud);
  cb = true;
  ignite();
}

void image_cb_ (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    threshold = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  im = true;
}

void ignite() {
  while(im && cb) {
    // std::cout << "both TRUE" << std::endl;
    for(int rows=0; rows<threshold.rows; rows++) {
      for(int cols=0; cols<threshold.cols; cols++) {
        int value = threshold.data[threshold.channels()*(threshold.cols*rows+cols)+0];
        if(value!=255) {
          // std::cout << "put zero" << std::endl;
          cloud->at(cols,rows).x = 0;
          cloud->at(cols,rows).y = 0;
          cloud->at(cols,rows).z = 0;
        }
      }
    }
    pub.publish(*cloud);
    im = false;
    cb = false;
  }
}
