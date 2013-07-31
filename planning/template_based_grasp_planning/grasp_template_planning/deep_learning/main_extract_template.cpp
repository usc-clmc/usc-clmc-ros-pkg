/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         main_extract_template.cpp

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#include <extract_template.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "/main_extract_template";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	return 0;
}
