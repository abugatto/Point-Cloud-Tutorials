#ifndef PCL_PRACTICE_CPP
#define PCL_PRACTICE_CPP

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
#include <pcl/filters/extract_indices.h>

//Project libraries
#include "paramHandler.hpp"
//#include "processCloud.hpp"

int i = 0;
pcl::visualization::PCLVisualizer viewer;

//Create ROS publisher object
ros::Publisher pub;

//Creates parameter object
Parameters* params = nullptr;

// void rvizNormals(const double& leafSize, 
// 				 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
// 				 pcl::PointCloud<pcl::Normal>::Ptr normals,
// 				 visualization_msgs::MarkerArray normalMarkers
// 				) {
//   	//apply extractIndices filter with voxelgrid according to frequency

//   	//Implement VoxelGrid Filter
// 	pcl::PCLPointCloud2Ptr cloudVoxelFiltered(new pcl::PCLPointCloud2);
// 	double leafSize = params->getLeafSize();

// 	pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGridFilter; //create voxelgrid object
// 	voxelGridFilter.setInputCloud(cloudBoxFilteredPtr);
// 	voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
// 	voxelGridFilter.filter(*cloudVoxelFiltered);

// 	pcl::PCLPointCloud2ConstPtr cloudVoxelFilteredPtr(cloudVoxelFiltered);

// 	ROS_INFO("VoxelGrid filter applied...");


//   	visualization_msgs::MarkerArray normals;
//   	for(int i = 0; i < cloudVoxelFilteredPtr->points.size(); i++) {
// 	  	//set normal parameters
// 	   	normals.marker[i].header.frame_id = params->getFrame();
// 		normals.marker[i].header.stamp = ros::Time();
// 		normals.marker[i].ns = "normals";
// 		normals.marker[i].id = 0;
// 		normals.marker[i].type = visualization_msgs::Marker::ARROW;
// 		normals.marker[i].action = visualization_msgs::Marker::ADD;



// 		//set normal scales
// 		normals.marker[i].scale.x = 0.01;
// 		normals.marker[i].scale.y = 0.025;
// 		normals.marker[i].scale.z = 0.01;

// 		//set normal colors
// 		normals.marker[i].color.a = 1.0;
// 		normals.marker[i].color.r = 0.0;
// 		normals.marker[i].color.g = 0.0;
// 		normals.marker[i].color.b = 1.0;
// 	}
// }

void pclviz(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
  	viewer.setBackgroundColor (0, 0, 0); //black
  	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud" + std::to_string(i));
  	//PCL_VISUALIZER_POINT_SIZE is int starting from 1
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + std::to_string(i));
  	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, .1, "normals" + std::to_string(i));
  	viewer.addCoordinateSystem (1.0);
  	viewer.initCameraParameters();
  	viewer.spinOnce();
  	i++;
}

//Define Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("Callback started...");

	//create a container for the data
	pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	//Convert to PCL
	pcl_conversions::toPCL(*input, *cloud);

	//Apply Radial Filter
	pcl::PCLPointCloud2Ptr cloudBoxFiltered(new pcl::PCLPointCloud2);
	double bound = params->getBoxFilterBound();

	pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-bound, -bound, -bound, 1.0));
	boxFilter.setMax(Eigen::Vector4f(bound, bound, bound, 1.0));
	boxFilter.setInputCloud(cloudPtr);
	boxFilter.filter(*cloudBoxFiltered);

	pcl::PCLPointCloud2ConstPtr cloudBoxFilteredPtr(cloudBoxFiltered);

	ROS_INFO("Box filter applied...");

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

  	// std::cout << cloud_normals->at(11) << std::endl;
  	// std::cout << "Cloud size is:" << cloud_normals->points.size() << std::endl;
  	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  	pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, indices->indices);
  	// std::cout << "New cloud size is:" << cloud_normals->points.size() << std::endl;
  	// std::cout << cloud_normals->at(11) << std::endl << std::endl;

  	//remove points from indices list
  	//could use this to display normals at each voxel
  	pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
  	extractIndices.setInputCloud(point_cloud);
  	extractIndices.setIndices(indices);
  	//extractIndices.setNegative(true);
  	extractIndices.filter(*point_cloud);
  	// std::cout << "Cloud size is " << point_cloud->points.size() << std::endl;

	//min number of normals

  	//visualize
  	// pcl::visualization::PCLVisualizer::Ptr viewer;
  	// pclviz(point_cloud, cloud_normals);

  	//DRAW ARROWS (have variable arrow size)
  	rvizNormals(point_cloud, cloud_normals)

	ROS_INFO("Surface normals found...");

	//Center Axis -> USE EIGEN
	//form similarity matrix from point curvatures in normal calculation


	//Form normal similarity matrix


	//Take eigenvalues of the similarity matrix


	//use minimum eigenvector as center axis and display on rviz


	//ROS_INFO("Center Axis found...");

	//Convert Back to ROS
	sensor_msgs::PointCloud2 output;

	pcl_conversions::fromPCL(*cloudBoxFiltered, output);

	//Publish the data
	pub.publish(output);

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

	//Calls message callbacks rapidly in seperate threads
	ros::spin();
}

#endif