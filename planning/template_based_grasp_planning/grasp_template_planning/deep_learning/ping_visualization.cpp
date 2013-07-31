/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         ping_visualization.cpp

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "ping_visualization";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>("/visualizer", 1000, 1);
	while (pub.getNumSubscribers() == 0) {
		ROS_ERROR("Waiting for subscibers");
		usleep(1000);
	}
	ROS_ERROR("Got subscriber");

	std_msgs::String msg;
	msg.data = "ping";
	pub.publish(msg);
	ros::spinOnce();
}
