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

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>


void Visualization_callback(const std_msgs::StringConstPtr &msg,Visualization &visualization){
	ROS_INFO("I heard: [%s]",msg->data.c_str());
	if(!visualization.Update_visualization()){
		ROS_ERROR("visualization is done, there is nothing more");
	}
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_visualization";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	Visualization visualization(nh);
	if(!visualization.Initilization()){
		ROS_ERROR("could not perform the initialization");
		return -1;
	}
	ros::Subscriber sub = nh.subscribe<std_msgs::String>("/visualizer",1000,boost::bind(Visualization_callback,_1,visualization));
	ros::spin();

	return 0;
}
