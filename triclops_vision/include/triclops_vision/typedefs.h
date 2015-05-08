/*
Copyright 2011, 2012 Massachusetts Institute of Technology

This work is sponsored by the Department of the Air Force under Air Force 
Contract #FA8721-05-C-0002. Opinions, interpretations, conclusions and 
recommendations are those of the authors and are not necessarily endorsed by the 
United States Government.
*/

#ifndef POINTCLOUD_REGISTRATION__TYPEDEFS_H
#define POINTCLOUD_REGISTRATION__TYPEDEFS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef pcl::PointIndices PointIndices;
typedef PointIndices::Ptr PointIndicesPtr;
typedef PointIndices::ConstPtr PointIndicesConstPtr;

typedef pcl::PointNormal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

typedef pcl::PointXYZINormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> NormalCloud;
typedef pcl::PointCloud<PointNormalT>::Ptr NormalCloudPtr;
typedef pcl::PointCloud<PointNormalT>::ConstPtr NormalCloudConstPtr;

typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

typedef pcl::VFHSignature308 GlobalDescriptorT;
typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;

#endif
