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

#include <deep_learning/visualization.h>
#include <deep_learning/log_loader.h>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

void Visualization_callback(const std_msgs::StringConstPtr &msg,
		Visualization *pvisualization,
		std::vector<grasp_template::GraspTemplate,
				Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	static unsigned int counter = 0;
	if (counter < result_template.size()) {
		ROS_WARN("start visualization");
		pvisualization->Update_visualization(
				result_template[counter]);
		counter += 1;
	} else {
		ROS_ERROR("visualization is done, there is nothing more");
	}
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_visualization";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	Visualization visualization(nh);
	std::string path_bagfile;
	if (!nh.getParam("bagfile_name", path_bagfile)) {
		ROS_ERROR("no bagfile name found");
		return -1;
	}
	Log_loader log_loader("/grasp_planning_log");
	std::vector<grasp_template::GraspTemplate,
			Eigen::aligned_allocator<grasp_template::GraspTemplate> > result_template;
	std::vector<std::string> result_uuid;
	std::vector<float> result_success;

	if (!log_loader.Load_trial_log(path_bagfile, result_template, result_uuid,
			result_success)) {
		ROS_ERROR("could not load and process bagfile %s",
				path_bagfile.c_str());
		return -1;
	}

	ros::Subscriber sub = nh.subscribe<std_msgs::String>("/visualizer", 1000,
			boost::bind(Visualization_callback, _1, &visualization,
					result_template));
	ROS_INFO("start listening to the publisher");
	ros::spin();

	return 0;
}
