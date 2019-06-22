#ifndef EXAMPLE_CPP
#define EXAMPLE_CPP

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//PCL libraries
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

//Project libraries
#include "paramHandler.h"

//Create ROS publisher object
ros::Publisher pub;

//Creates parameter object
Parameters* params;

//Define Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	//create a container for the data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloudNew;

	//TEST CODE
	if(params->getDebug) {
		std::cout << "\n\n\nparams->applyBoxFilter: " << params->getApplyBoxFilter() << std::endl;
		std::cout << "params->boxFilterDist: " << params->getBoxFilterDist() << std::endl;
		std::cout << "params->applyVoxelGridFilter: " << params->getApplyVoxelGridFilter() << std::endl;
		std::cout << "params->leafSize: " << params->getLeafSize() << std::endl;
		std::cout << "params->findSurfaceNormals: " << params->getFindSurfaceNormals() << std::endl;
		std::cout << "params->neighborRadius: " << params->getNeighborRadius() << std::endl;
	}

	//Convert to PCL
	pcl_conversions::toPCL(*input, *cloud);

	//Apply Box Filter
	if(params->getApplyBoxFilter()) {


		ROS_DEBUG("Box filter applied...");
	}

	//Implement VoxelGrid Filter
	if(params->getApplyVoxelGridFilter()) {
		//std::cout << cloud->header << std::endl; TEST CODE
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //create voxelgrid object
		sor.setInputCloud(cloudPtr);
		sor.setLeafSize(params->getLeafSize(), params->getLeafSize(), params->getLeafSize());
		sor.filter(cloudNew);

		ROS_DEBUG("VoxelGrid filter applied...");
	} 

	//Find Surface Normals
	if(params->getFindSurfaceNormals()) {
		// Create the normal estimation class
  		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> find_normals;
  		find_normals.setInputCloud(cloud);

  		//create KD tree of the point cloud
  		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  		find_normals.setSearchMethod(tree);

  		//Find normals using nearest neighbor parameter
  		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  		find_normals.setRadiusSearch(params->getNeighborRadius());
  		find_normals.compute(*cloud_normals);

  		//Should I publish multiple clouds: filtered cloud and surface normal cloud???

		ROS_DEBUG("Surface normals found...");
	}
	
	if(params->getDebug) {
		cloudNew = *cloud;
		ROS_DEBUG_STREAM(input->header);
	}

	//Convert Back to ROS
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloudNew, output);

	//Publish the data
	pub.publish(output);
}

//main function creates subscriber for the published point cloud
int main(int argc, char** argv) {
	//Initialize ROS
	ros::init(argc, argv, "pcl_practice");
	ros::NodeHandle node;

	//Parameters params;
	*params = Parameters(node);

	if(params->getDebug) {
		std::cout<<"\n\n\nNODE launched\n"; //TEST CODE
	}

	//Create subscriber for the input pointcloud
	ros::Subscriber sub = node.subscribe("/velodyne_points", 1, cloud_cb);

	//Create ROS publisher for point cloud
	pub = node.advertise<sensor_msgs::PointCloud2>("output", 1);

	//Calls message callbacks rapidly
	ros::spin();
}

#endif