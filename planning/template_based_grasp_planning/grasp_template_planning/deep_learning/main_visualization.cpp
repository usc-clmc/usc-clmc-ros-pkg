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
#include <deep_learning/data_loader.h>
#include <deep_learning/data_grasp.h>
#include <deep_learning/extract_template.h>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

bool Visualization_callback(std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response, Visualization *pvisualization,
		std::vector<Data_grasp> &result_grasp) {
	static unsigned int counter = 0;
	Eigen::Vector3d dim(0.21, 0.22, 0.28);
	if (counter < result_grasp.size()) {
		Data_grasp temp = result_grasp[counter];
		ROS_WARN("start visualization");
		ROS_INFO("result_uuid    %s", temp.uuid.c_str());
		ROS_INFO("result_success %f", temp.success);
		pvisualization->Update_visualization(temp.grasp_template);
		geometry_msgs::Pose temp_pose;
		temp.grasp_template.getPose(temp_pose);
		geometry_msgs::Pose result_pose;
		Extract_template::Coordinate_to_world(temp_pose,temp.gripper_pose,result_pose);
		//temp_pose.orientation += temp.gripper_pose.orientation;
		pvisualization->Update_cube(result_pose, dim);
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
	Data_loader data_loader("/grasp_planning_log");
	std::vector<Data_grasp> result_grasp;
	sensor_msgs::PointCloud2 result_object_cloud;
	geometry_msgs::Pose result_view_point;

	if (!data_loader.Load_trial_log(path_bagfile, result_grasp,
			result_object_cloud, result_view_point)) {
		ROS_ERROR("could not load and process bagfile %s",
				path_bagfile.c_str());
		return -1;
	}

	visualization.Update_visualization(result_object_cloud);
	visualization.Update_pose(result_view_point);

	ROS_INFO("waiting for service requests");

	ros::ServiceServer service = nh.advertiseService<std_srvs::EmptyRequest,
			std_srvs::EmptyResponse>("deep_learning_test",
			boost::bind(Visualization_callback, _1, _2, &visualization,
					result_grasp));
	ros::spin();

	return 0;
}
