/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         main-visualization.cpp

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#include <visualization.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "/main_visualization";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	Visualization visualization(nh);
	grasp_template::HsIterator hs_iter = visualization.Initilization();
	if(hs_iter.passedLast()){
		ROS_ERROR("could not perform the initialization");
	}
	while (visualization.Update_visualization(hs_iter) && ros::ok()){
		usleep(1000000);
	}
	return 0;
}
