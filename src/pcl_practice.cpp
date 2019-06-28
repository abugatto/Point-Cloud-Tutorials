#ifndef PCL_PRACTICE_CPP
#define PCL_PRACTICE_CPP

#include <thread>

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

//PCL libraries
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

//Project libraries
#include "paramHandler.hpp"
//#include "processCloud.hpp"

int i = 0;
pcl::visualization::PCLVisualizer viewer;

//Create ROS publisher object
ros::Publisher pub;

//Creates parameter object
Parameters* params = nullptr;

void pclviz(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
  	viewer.initCameraParameters();
  	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud" + std::to_string(i));
  	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 2, 0.5, "normals" + std::to_string(i));
  	i++;
  	std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

//Define Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("Callback started...");

	//create a container for the data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	//Convert to PCL
	pcl_conversions::toPCL(*input, *cloud);

	//Apply Radial Filter
	pcl::PCLPointCloud2* cloudBoxFiltered = new pcl::PCLPointCloud2;
	double bound = params->getBoxFilterBound();

	pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-bound, -bound, -bound, 1.0));
	boxFilter.setMax(Eigen::Vector4f(bound, bound, bound, 1.0));
	boxFilter.setInputCloud(cloudPtr);
	boxFilter.filter(*cloudBoxFiltered);

	pcl::PCLPointCloud2ConstPtr cloudBoxFilteredPtr(cloudBoxFiltered);

	ROS_INFO("Box filter applied...");

	//Implement VoxelGrid Filter
	pcl::PCLPointCloud2* cloudVoxelFiltered = new pcl::PCLPointCloud2;
	double leafSize = params->getLeafSize();

	pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGridFilter; //create voxelgrid object
	voxelGridFilter.setInputCloud(cloudBoxFilteredPtr);
	voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
	voxelGridFilter.filter(*cloudVoxelFiltered);

	pcl::PCLPointCloud2ConstPtr cloudVoxelFilteredPtr(cloudVoxelFiltered);

	ROS_INFO("VoxelGrid filter applied...");

	//Find Surface Normals
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloudBoxFiltered, *point_cloud);

	//Create the normal estimation class
  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> find_normals;
  	find_normals.setInputCloud(point_cloud);

  	//create KD tree of the point cloud
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  	find_normals.setSearchMethod(tree);

  	//Find normals using nearest neighbor parameter
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  	find_normals.setRadiusSearch(params->getNeighborRadius());
  	find_normals.compute(*cloud_normals);

  	ROS_INFO(cloud_normals->at(11));

	//filter nan normals

  	//visualize
  	pcl::visualization::PCLVisualizer::Ptr viewer;
  	pclviz(point_cloud, cloud_normals);

  	//DRAW ARROWS (have variable arrow size)
  	// visualization_msgs::MarkerArray normals;
  	// for(int i = 0; i < cloudVoxelFilteredPtr->points.size(); i++) {
	//   	//set normal parameters
	//    	normals.marker[i].header.frame_id = params->getFrame();
	// 	normals.marker[i].header.stamp = ros::Time();
	// 	normals.marker[i].ns = "normals";
	// 	normals.marker[i].id = 0;
	// 	normals.marker[i].type = visualization_msgs::Marker::ARROW;
	// 	normals.marker[i].action = visualization_msgs::Marker::ADD;



	// 	//set normal scales
	// 	normals.marker[i].scale.x = 0.01;
	// 	normals.marker[i].scale.y = 0.025;
	// 	normals.marker[i].scale.z = 0.01;

	// 	//set normal colors
	// 	normals.marker[i].color.a = 1.0;
	// 	normals.marker[i].color.r = 0.0;
	// 	normals.marker[i].color.g = 0.0;
	// 	normals.marker[i].color.b = 1.0;
	// }

	ROS_INFO("Surface normals found...");

	ROS_INFO_STREAM(input->header);

	//Center Axis


	//Convert Back to ROS
	sensor_msgs::PointCloud2 output;
	if(params->getApplyBoxFilter()) {
		pcl_conversions::fromPCL(*cloudBoxFiltered, output);

		//Publish the data
		pub.publish(output);
	} else if(params->getApplyVoxelGridFilter()) {
		pcl_conversions::fromPCL(*cloudVoxelFiltered, output);

		//Publish the data
		pub.publish(output);
	} else if(params->getFindSurfaceNormals()) {
		//Publish marker array
	} else {
		pcl_conversions::fromPCL(*cloudVoxelFiltered, output);

		//Publish the data
		pub.publish(output);
	}

	ROS_INFO("Published...");
}

//main function creates subscriber for the published point cloud
int main(int argc, char** argv) {
	//Initialize ROS
	ros::init(argc, argv, "pcl_practice_node");
	ros::NodeHandle node;

	//Parameters params;
	Parameters param(node);
	params = &param;

	ROS_INFO("Launched pcl_practice_node...");

	//Create subscriber for the input pointcloud
	ros::Subscriber sub = node.subscribe("input", 1, cloud_cb);

	//Create ROS publisher for point cloud
	if(params->getFindSurfaceNormals()) {
		pub = node.advertise<visualization_msgs::MarkerArray>("output", 10);
	} else {
		pub = node.advertise<sensor_msgs::PointCloud2>("output", 10);
	}

	//Calls message callbacks rapidly
	ros::spin();
	viewer.spin();
}

#endif