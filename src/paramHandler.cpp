#ifndef PARAMHANDLER_CPP
#define PARAMHANDLER_CPP

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//.h File
#include "paramHandler.h"

//Deals with the parameter data
void Parameters::Parameters(ros::NodeHandle& nd) {
	node = nd;

	if(ros::param::get("debug", debug)) {
		ROS_DEBUG("debug set to:\t %d", debug);
	} else {
		ROS_DEBUG("ERROR: debug set to default...");
	}

	if(ros::param::get("applyBoxFilter", applyBoxFilter)) {
		ROS_DEBUG("applyBoxFilter set to:\t %d", applyBoxFilter);
	} else {
		ROS_DEBUG("ERROR: applyBoxFilter set to default...");
	}

	if(ros::param::get("boxFilterDist", boxFilterDist)) {
		ROS_DEBUG("boxFilterDist set to:\t %f", boxFilterDist);
	} else {
		ROS_DEBUG("ERROR: boxFilterDist set to default...");
	}

	if(ros::param::get("applyVoxelGridFilter", applyVoxelGridFilter)) {
		ROS_DEBUG("applyVoxelGridFilter set to:\t %d", applyVoxelGridFilter);
	} else {
		ROS_DEBUG("ERROR: applyVoxelGridFilter set to default...");
	}

	if(ros::param::get("leafSize", leafSize)) {
		ROS_DEBUG("leafSize set to:\t %f", leafSize);
	} else {
		ROS_DEBUG("ERROR: leafSize set to default...");
	}

	if(ros::param::get("findSurfaceNormals", findSurfaceNormals)) {
		ROS_DEBUG("findSurfaceNormals set to:\t %d", findSurfaceNormals);
	} else {
		ROS_DEBUG("ERROR: findSurfaceNormals set to default...");
	}

	if(ros::param::get("neighborRadius", neighborRadius)) {
		ROS_DEBUG("neighborRadius set to:\t %f", neighborRadius);
	} else {
		ROS_DEBUG("ERROR: neighborRadius set to default...");
	}
}

bool getDebug() {
	return debug;
}

bool getApplyBoxFilter() {
	return ApplyBoxFilter;
} 

double getBoxFilterDist() {
	return getBoxFilterDist;
}

bool getApplyVoxelGridFilter() {
	return applyVoxelGridFilter;
} 

double getLeafSize() {
	return leafSize;
}  

bool getFindSurfaceNormals() {
	return findSurfaceNormals;
} 

double getNeighborRadius() {
	return neighborRadius;
}
