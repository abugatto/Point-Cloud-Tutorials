#ifndef PROCESSCLOUD_HPP
#define PROCESSCLOUD_HPP

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

//cloud processing class
class processCloud {
	public:
		processCloud(const sensor_msgs::PointCloud2ConstPtr& input);
		processCloud(const pcl::PointCloud2 input);
		bool getCloudMsg(sensor_msgs::PointCloud2& cloudMsg);
		bool getPCLCloud(pcl::PointCloud2& cloud);


	private:

}

#endif