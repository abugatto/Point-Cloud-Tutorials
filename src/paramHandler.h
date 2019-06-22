#ifndef PARAMHANDLER_H
#define PARAMHANDLER_H

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//Parameter Handler Class
class Parameters {
	public:
		Parameters(ros::NodeHandle& node);
		bool getDebug();
		bool getApplyBoxFilter();
		double getBoxFilterDist(); 
		bool getApplyVoxelGridFilter();
		double getLeafSize();
		bool getFindSurfaceNormals();
		bool getNeighborRadius(); 

	private:
		ros::NodeHandle node;
		bool debug = true;
		bool applyBoxFilter = true;
		double boxFilterDist = 5.0;
		bool applyVoxelGridFilter = true;
		double leafSize = .1;
		bool findSurfaceNormals = true;
		bool neighborRadius = .03;
};

//MAKE CLASS FOR POINT CLOUD HANDLING

#endif