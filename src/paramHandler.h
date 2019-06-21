#ifndef PARAMHANDLER_H
#define PARAMHANDLER_H

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//Parameter Handler Class
class Parameters {
	public:
		Parameters(ros::NodeHandle& node);

	private:
		ros::NodeHandle node;
		bool applyBoxFilter = true;
		double boxFilterDist = 5.0;
		bool applyVoxelGridFilter = true;
		double leafSize = .1;
		bool findSurfaceNormals = true;
		bool neighborRadius = .03;
};

//MAKE CLASS FOR POINT CLOUD HANDLING

#endif