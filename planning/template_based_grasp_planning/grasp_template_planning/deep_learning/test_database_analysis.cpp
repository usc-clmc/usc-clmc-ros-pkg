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

#include <deep_learning/data_storage.h>
#include <deep_learning/data_loader.h>
#include <deep_learning/data_grasp.h>
#include <deep_learning/def.h>
#include <deep_learning/visualization.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace fs = boost::filesystem3;
using namespace deep_learning;

bool Store_grasp(Data_grasp &data_grasp, fs::path dir_path_results) {
	std::stringstream ss;
	ss << data_grasp.uuid_database << "_";
	if (data_grasp.success == SUCCESS_TRUE) {
		ss << "success";
	}
	if (data_grasp.success == SUCCESS_FALSE) {
		ss << "fail";
	}
	fs::path dir_path_results_local = dir_path_results / fs::path(ss.str());
	if (!fs::is_directory(dir_path_results_local)) {
		fs::create_directory(dir_path_results_local);
	}

	cv::Mat tmp_image = Visualization::Render_image_solid(
			data_grasp.grasp_template.heightmap_);

	ss.clear();
	ss << data_grasp.uuid_database_template;
	fs::path file_path_result = dir_path_results_local
			/ fs::path(ss.str() + "--solid.bmp");
	if (tmp_image.empty()) {
		return true;
	}
	cv::imwrite(file_path_result.c_str(), tmp_image);
	return true;
}

bool Visualization_callback(std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response, Visualization *pvisualization,
		std::vector<Data_grasp> &result_grasp) {
	static unsigned int counter = 0;
	// arm cube
	//Eigen::Vector3d dim(0.21, 0.22, 0.28);
	// pr2 cube
	//Eigen::Vector3d dim(0.165, 0.24, 0.09);
	//Eigen::Vector3d dim(0.165, 0.09, 0.24);
	//Eigen::Vector3d dim(0.24, 0.165, 0.09);
	if (counter < result_grasp.size()) {
		Data_grasp temp = result_grasp[counter];
		while((temp.success == SUCCESS_TRUE) && (counter < result_grasp.size())){
			counter += 1;
			temp = result_grasp[counter];
		}
		ROS_WARN("start visualization");
		std::cout << " success " << temp.success << std::endl;
		pvisualization->Update_visualization(temp.grasp_template);
		geometry_msgs::Pose temp_pose;
		temp.grasp_template.getPose(temp_pose);
		geometry_msgs::Pose result_pose;

		Extract_template::Coordinate_to_world(temp_pose, temp.gripper_pose,
				result_pose);
		//temp_pose.orientation += temp.gripper_pose.orientation;
		//pvisualization->Update_cube(result_pose, dim);
		counter += 1;
	} else {
		ROS_ERROR("visualization is done, there is nothing more");
	}
	return true;
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "test_database_analysis";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	std::string _dir_path_base;
	std::string _dir_path_results;
	std::string _dir_path_database;
	std::string _database_name;

	if (!nh.getParam("dir_path_base", _dir_path_base)) {
		ROS_ERROR("dir path base");
		return -1;
	}
	if (!nh.getParam("dir_path_database", _dir_path_database)) {
		ROS_ERROR("dir path database");
		return -1;
	}
	if (!nh.getParam("dir_path_results", _dir_path_results)) {
		ROS_ERROR("dir path bagfiles");
		return -1;
	}
	if (!nh.getParam("database_name", _database_name)) {
		ROS_ERROR("database name");
		return -1;
	}

	try {
		fs::path dir_path_base(_dir_path_base);

		fs::path dir_path_results = dir_path_base / fs::path(_dir_path_results);
		if (!fs::is_directory(dir_path_results)) {
			fs::create_directory(dir_path_results);
		}

		Data_loader data_loader;
		Data_storage data_storage;

		fs::path dir_path_database = dir_path_base
				/ fs::path(_dir_path_database);
		// have to get the path for the database
		data_storage.Init_database(dir_path_database.c_str(), _database_name);
		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;
		std::cout << "load database" << std::endl;
		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;
		Database_grasp database_grasp = data_loader.Load_grasp_database(
				data_storage.Get_file_path_database().c_str());

		std::vector<Data_grasp> result_grasps;
		database_grasp.Get_grasps(result_grasps);

		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;
		std::cout << "loaded database coutains " << result_grasps.size()
				<< " grasps" << std::endl;
		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;

		for (unsigned int i = 0; i < result_grasps.size(); ++i) {
			Store_grasp(result_grasps[i], dir_path_results);
		}

		Visualization visualization(nh);

		ros::ServiceServer service = nh.advertiseService<std_srvs::EmptyRequest,
				std_srvs::EmptyResponse>("deep_learning_test",
				boost::bind(Visualization_callback, _1, _2, &visualization,
						result_grasps));
		ros::spin();

	} catch (const fs::filesystem_error &ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}
