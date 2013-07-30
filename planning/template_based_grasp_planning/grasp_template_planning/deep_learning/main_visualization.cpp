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
	ros::init(argc, argv, "main_visualization");
	ros::NodeHandle nh;
	Visualization visualization(nh);
	grasp_template::HsIterator hs_iter = visualization.Initilization();
	while (visualization.Update_visualization(hs_iter) && ros::ok()){
		usleep(1000000);
	}
	return 0;
}
