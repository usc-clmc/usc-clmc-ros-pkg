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
#include <map>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <iostream>

namespace fs = boost::filesystem3;
namespace po = boost::program_options;
using namespace deep_learning;

bool Visualization_callback(std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response, Visualization *pvisualization,
		std::vector<Data_grasp> &result_grasp) {
	static unsigned int counter = 0;
	Eigen::Vector3d dim(0.21, 0.22, 0.28);
	if (counter < result_grasp.size()) {
		Data_grasp temp = result_grasp[counter];
		ROS_WARN("start visualization");
		ROS_INFO("result_uuid    %i", temp.uuid_database);
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

bool Parse_log(Data_loader &data_loader, Data_storage &data_storage,
		std::string path_bagfile) {
	std::vector<Dataset_grasp> result_dataset_grasps;
	std::cout << path_bagfile << std::endl;
	if (!data_loader.Load_unsupervised_from_grasp_log(path_bagfile, "/grasp_planning_log",
			result_dataset_grasps)) {
		ROS_ERROR("could not load and process bagfile %s",
				path_bagfile.c_str());
		return false;
	}
	for (unsigned int i = 0; i < result_dataset_grasps.size(); ++i) {
		data_storage.Update_dataset(result_dataset_grasps[i]);
	}
	/*
	const std::string ROS_NH_NAME_SPACE = "main_grasp_data_dataset";
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);

	Visualization visualization(nh);
	ros::ServiceServer service = nh.advertiseService<std_srvs::EmptyRequest,
			std_srvs::EmptyResponse>("deep_learning_test",
			boost::bind(Visualization_callback, _1, _2, &visualization,
					local_result_grasps));
	ros::spin();
	*/
	return true;
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_grasp_data_unsupervised_grasp_log";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	std::string _dir_path_bagfiles;
	std::string _dir_path_database;
	std::string _dir_path_dataset;
	std::string _database_name;
	std::string _dataset_name;
	if (!nh.getParam("dir_path_bagfiles", _dir_path_bagfiles)) {
		ROS_ERROR("dir path bagfiles");
		return -1;
	}
	if (!nh.getParam("dir_path_database", _dir_path_database)) {
		ROS_ERROR("dir path database");
		return -1;
	}
	if (!nh.getParam("database_name", _database_name)) {
		ROS_ERROR("database name");
		return -1;
	}
	if (!nh.getParam("dir_path_dataset", _dir_path_dataset)) {
		ROS_ERROR("dir path dataset");
		return -1;
	}
	if (!nh.getParam("dataset_name", _dataset_name)) {
		ROS_ERROR("database name");
		return -1;
	}

	try {
		fs::path dir_path_bagfiles(_dir_path_bagfiles);
		if (!fs::exists(dir_path_bagfiles)) {
			std::cout << "path does not exist " << dir_path_bagfiles.c_str()
					<< std::endl;
		}
		if (!fs::is_directory(dir_path_bagfiles)) {
			std::cout << "path is not a directory" << dir_path_bagfiles.c_str()
					<< std::endl;
		}

		Data_loader data_loader;
		Data_storage data_storage;

		// load the stored templates
		std::cout << "init database" << std::endl;
		data_storage.Init_database(_dir_path_database,_database_name);
		std::cout << "init dataset" << std::endl;
		data_storage.Init_dataset(_dir_path_dataset,_dataset_name);
		std::vector<Data_grasp> grasp_templates;
		std::cout << "load database" << std::endl;
		data_loader.Load_grasp_database(
				data_storage.Get_file_path_database().c_str(),
				"/deep_learning_data_grasp_database", grasp_templates);

		std::cout << "init loader dataset" << std::endl;
		data_loader.Init_dataset(grasp_templates);

		fs::directory_iterator it(dir_path_bagfiles), eod;

		int counter = 0;
		BOOST_FOREACH(fs::path const &p_tmp, std::make_pair(it, eod)){
		if(fs::is_regular_file(p_tmp))
		{
			Parse_log(data_loader,data_storage,p_tmp.c_str());
			///* debug start
			counter +=1;
			if(counter >= 2){
				std::cout << " debug stop " << std::endl;
				break;
			}
			// debug end
			//*/
		}


	}
		data_storage.Store_dataset();
} catch (const fs::filesystem_error &ex) {
	std::cout << ex.what() << std::endl;
}

	return 0;
}
