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

#include <ros/ros.h>
#include <map>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <iostream>

namespace fs = boost::filesystem3;
namespace po = boost::program_options;
using namespace deep_learning;

bool Parse_log(Data_loader &data_loader, std::string path_bagfile,
		std::map<std::size_t, Data_grasp> &result_database_grasps,std::map<std::size_t, Dataset_grasp> &result_dataset_grasps) {
	Data_grasp g_tmp_database = data_loader.Load_grasp_database(path_bagfile,"/grasp_planning_log");
	Dataset_grasp g_tmp_dataset = data_loader.Load_grasp_dataset(path_bagfile,"/grasp_planning_log");
	if (g_tmp_database.uuid_database == 0) {
		return false;
	}
	result_database_grasps.insert(std::make_pair(g_tmp_database.uuid_database, g_tmp_database));
	result_dataset_grasps.insert(std::make_pair(g_tmp_dataset.uuid_database, g_tmp_dataset));
	return true;
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_data_grasp_database";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);

	po::options_description desc(
			"Extract grasp database from alex's bag files.");

	std::string _dir_path_bagfiles;
	std::string _dir_path_database;
	std::string _database_name;

	desc.add_options()("help", "produce help message")("dir-path-bagfiles",
			po::value<std::string>(&_dir_path_bagfiles)->required(),
			"path to the directory of the bagfiles")("dir-path-database",
			po::value<std::string>(&_dir_path_database)->required(),
			"path to the file where the grasp database should be stored")("database-name", po::value<
			std::string>(&_database_name)->required(), "name of the database");
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}
	po::notify(vm);

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
		std::vector<Data_grasp> result_grasp;
		std::map<std::size_t, Data_grasp> result_database_grasps;
		std::map<std::size_t, Dataset_grasp> result_dataset_grasps;

		fs::directory_iterator it(dir_path_bagfiles), eod;

		BOOST_FOREACH(fs::path const &p_tmp, std::make_pair(it, eod)){
		if(fs::is_regular_file(p_tmp))
		{
			Parse_log(data_loader,p_tmp.c_str(),result_database_grasps,result_dataset_grasps);
		}

	}

		data_storage.Init_database(_dir_path_database,_database_name);
		data_storage.Store_database(result_database_grasps);
		data_storage.Init_dataset(_dir_path_database,_database_name+"-dataset");
		data_storage.Update_dataset(result_dataset_grasps);
		data_storage.Store_dataset();
		std::vector<Data_grasp> test_grasps;
		data_loader.Load_grasp_database(
				data_storage.Get_file_path_database().c_str(),"/deep_learning_data_grasp_database", test_grasps);
		std::cout << " test grasps " << std::endl;
		for (unsigned int i = 0; i < test_grasps.size(); ++i) {
			std::cout << test_grasps[i].uuid_database << std::endl;
			std::cout << test_grasps[i].gripper_pose << std::endl;
			std::cout << "gripper joints";
			for (unsigned int u = 0; u < test_grasps[i].gripper_joints.size(); ++u) {
			std::cout << test_grasps[i].gripper_joints[u] << ", ";
			}
			std::cout << std::endl;
		}

	} catch (const fs::filesystem_error &ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}
