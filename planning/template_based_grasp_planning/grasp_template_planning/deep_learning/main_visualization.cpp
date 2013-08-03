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
#include <std_srvs/Empty.h>

bool Visualization_callback(std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response, Visualization *pvisualization,
		std::vector<grasp_template::GraspTemplate,
				Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
		std::vector<std::string> &result_uuid,
		std::vector<float> &result_success) {
	static unsigned int counter = 0;
	if (counter < result_template.size()) {
		ROS_WARN("start visualization");
		ROS_INFO("result_uuid    %s", result_uuid[counter].c_str());
		ROS_INFO("result_success %f", result_success[counter]);
		pvisualization->Update_visualization(result_template[counter]);
		counter += 1;
	} else {
		ROS_ERROR("visualization is done, there is nothing more");
	}
	return true;
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

	ros::ServiceServer service = nh.advertiseService<std_srvs::EmptyRequest,std_srvs::EmptyResponse>("deep_learning_test",
			boost::bind(Visualization_callback, _1, _2, &visualization,
					result_template, result_uuid, result_success));
	ros::spin();

	return 0;
}
