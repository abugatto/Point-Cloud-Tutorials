#ifndef PARAMHANDLER_H
#define PARAMHANDLER_H

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

class Parameters {
	public:
		Parameters(ros::NodeHandle& node);

	private:
		ros::NodeHandle node;
		bool applyVoxelGridFilter = true;
		double leafSize = .1;
		bool findSurfaceNormals = true;
};

#endif