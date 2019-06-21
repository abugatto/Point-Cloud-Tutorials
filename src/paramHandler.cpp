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
	// nh_.getParam("/my_node/applyVoxelGridFilter",params.applyVoxelGridFilter);
	// nh_.getParam("voxelGridLeafSize",params.leafSize);
	// nh_.getParam("findSurfaceNormals",params.findSurfaceNormals);

	if(ros::param::get("applyBoxFilter", params.applyBoxFilter)) {
		ROS_DEBUG("applyBoxFilter set to:\t %d", params.applyBoxFilter);
	} else {
		ROS_DEBUG("ERROR: applyBoxFilter set to default...");
	}

	if(ros::param::get("boxFilterDist", params.boxFilterDist)) {
		ROS_DEBUG("boxFilterDist set to:\t %f", params.boxFilterDist);
	} else {
		ROS_DEBUG("ERROR: boxFilterDist set to default...");
	}

	if(ros::param::get("applyVoxelGridFilter", params.applyVoxelGridFilter)) {
		ROS_DEBUG("applyVoxelGridFilter set to:\t %d", params.applyVoxelGridFilter);
	} else {
		ROS_DEBUG("ERROR: applyVoxelGridFilter set to default...");
	}

	if(ros::param::get("leafSize", params.leafSize)) {
		ROS_DEBUG("leafSize set to:\t %f", params.leafSize);
	} else {
		ROS_DEBUG("ERROR: leafSize set to default...");
	}

	if(ros::param::get("findSurfaceNormals", params.findSurfaceNormals)) {
		ROS_DEBUG("findSurfaceNormals set to:\t %d", params.findSurfaceNormals);
	} else {
		ROS_DEBUG("ERROR: findSurfaceNormals set to default...");
	}

	if(ros::param::get("neighborRadius", params.neighborRadius)) {
		ROS_DEBUG("neighborRadius set to:\t %f", params.neighborRadius);
	} else {
		ROS_DEBUG("ERROR: neighborRadius set to default...");
	}
}