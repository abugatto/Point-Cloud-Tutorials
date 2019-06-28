#ifndef PROCESSCLOUD_CPP
#define PROCESSCLOUD_CPP

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//PCL libraries
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

//Header File
#include "processCloud.hpp"


#endif