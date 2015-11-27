#ifndef TYPES_H
#define TYPES_H

#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
// Helper typedefs to make the implementation code cleaner
/*
 * Simple Point Cloud Type
 */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef typename PointCloudT::Ptr PointCloudTPtr;
typedef typename PointCloudT::ConstPtr PointCloudTConstPtr;

/*
 * Colored Point Cloud Type
 */
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointCT> PointCloudCT;
typedef typename PointCloudCT::Ptr PointCloudCTPtr;
typedef typename PointCloudCT::ConstPtr PointCloudCTConstPtr;

/*
 * Point Cloud Normal Type
 */
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
/*
 * Point Cloud Feature Type
 */
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

#endif // TYPES_H
