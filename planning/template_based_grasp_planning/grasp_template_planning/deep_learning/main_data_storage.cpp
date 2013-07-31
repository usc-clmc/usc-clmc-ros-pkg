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

#include <data_storage.h>
#include <visualization.h>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>


void Store_callback(const std_msgs::StringConstPtr &msg,Data_storage *pdata_storage){
	ROS_INFO("I heard: [%s]",msg->data.c_str());
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_data_storage";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);

	Data_storage data_storage("/tmp/test-data-storage");

	ros::Subscriber sub = nh.subscribe<std_msgs::String>("/visualizer",1000,boost::bind(Store_callback,_1,&data_storage));
	ros::spin();

	return 0;
}
