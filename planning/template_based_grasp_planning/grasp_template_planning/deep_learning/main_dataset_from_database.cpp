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
#include <deep_learning/extract_template.h>
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

bool Parse_log(Data_loader &data_loader, Data_storage &data_storage,
		Extract_template &extract_tempalte, std::string path_bagfile) {
	std::vector<Dataset_grasp> result_dataset_grasps;
	std::cout << path_bagfile << std::endl;
	if (!data_loader.Load_from_grasp_log(path_bagfile, "/grasp_planning_log",
			extract_tempalte, result_dataset_grasps)) {
		ROS_ERROR("could not load and process bagfile %s",
				path_bagfile.c_str());
		return false;
	}
	for (unsigned int i = 0; i < result_dataset_grasps.size(); ++i) {
		data_storage.Update_dataset(result_dataset_grasps[i]);
	}
	return true;
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_dataset_from_database";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);

	std::string _dir_path_base;
	std::string _dir_path_database;
	std::string _database_name;
	std::string _dataset_name;

	if (!nh.getParam("dir_path_base", _dir_path_base)) {
		ROS_ERROR("dir path base");
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
	if (!nh.getParam("dataset_name", _dataset_name)) {
		ROS_ERROR("database name");
		return -1;
	}

	try {
		fs::path dir_path_base(_dir_path_base);
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

		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;
		std::cout << "loaded database" << std::endl;
		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;

		std::vector<Data_grasp> grasp_library;
		database_grasp.Get_grasps(grasp_library);
		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;
		std::cout << "load " << grasp_library.size() << " templates"
				<< std::endl;
		std::cout
				<< "///////////////////////////////////////////////////////////////////"
				<< std::endl;

		fs::path dir_path_dataset = dir_path_base
				/ fs::path(_dir_path_database);

		if (!fs::is_directory(dir_path_dataset)) {
			fs::create_directories(dir_path_dataset);
		}

		data_storage.Init_dataset(dir_path_dataset.c_str(), _dataset_name);

			std::vector<Dataset_grasp> result_dataset_grasp;
		for (unsigned int i = 0; i < grasp_library.size(); ++i) {
			Dataset_grasp::Add(grasp_library[i],
					result_dataset_grasp);
		}

			for (unsigned int j = 0; j < result_dataset_grasp.size(); ++j) {
				data_storage.Update_dataset(result_dataset_grasp[j]);
			}
		data_storage.Store_dataset();
	} catch (const fs::filesystem_error &ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}
