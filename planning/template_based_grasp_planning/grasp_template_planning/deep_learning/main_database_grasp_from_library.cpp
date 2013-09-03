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

bool Parse_log(Data_loader &data_loader, Database_grasp &database_grasp,
		std::string path_bagfile,int success) {
	std::vector<Data_grasp> result_data_grasps;
	std::cout << path_bagfile << std::endl;
	std::cout << "load bag" << std::endl;
	if (!data_loader.Load_from_library(path_bagfile,
			result_data_grasps,success)) {
		ROS_ERROR("could not load and process bagfile %s",
				path_bagfile.c_str());
		return false;
	}
	for (unsigned int i = 0; i < result_data_grasps.size(); ++i) {
		database_grasp.Add_grasp(result_data_grasps[i]);
	}
	return true;
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_database_grasp_from_library";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	std::string _dir_path_base;
	std::string _dir_path_bagfiles;
	std::string _dir_path_database;
	std::string _database_name;
	int success;

	if (!nh.getParam("dir_path_base", _dir_path_base)) {
		ROS_ERROR("dir path base");
		return -1;
	}
	if (!nh.getParam("dir_path_database", _dir_path_database)) {
		ROS_ERROR("dir path database");
		return -1;
	}
	if (!nh.getParam("dir_path_bagfiles", _dir_path_bagfiles)) {
		ROS_ERROR("dir path bagfiles");
		return -1;
	}
	if (!nh.getParam("database_name", _database_name)) {
		ROS_ERROR("database name");
		return -1;
	}
	if (!nh.getParam("success", success)) {
		ROS_ERROR("success");
		return -1;
	}

	try {
		fs::path dir_path_base(_dir_path_base);
		fs::path dir_path_bagfiles = dir_path_base/fs::path(_dir_path_bagfiles);
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

		fs::path dir_path_database = dir_path_base/fs::path(_dir_path_database);
		// have to get the path for the database
		data_storage.Init_database(dir_path_database.c_str(), _database_name);
		std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;
		std::cout << "load database" << std::endl;
		std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;
		Database_grasp database_grasp = data_loader.Load_grasp_database(data_storage.Get_file_path_database().c_str());

		std::vector<Data_grasp> result_grasps;
		database_grasp.Get_grasps(result_grasps);

		std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;
		std::cout << "loaded database coutains " << result_grasps.size() << " grasps" << std::endl;
		std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;

		fs::directory_iterator it(dir_path_bagfiles), eod;


		//int counter = 0;
		BOOST_FOREACH(fs::path const &p_tmp, std::make_pair(it, eod)){
		if(fs::is_regular_file(p_tmp))
		{

		std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;
		std::cout << "load from library" << std::endl;
		std::cout << "///////////////////////////////////////////////////////////////////" << std::endl;
			Parse_log(data_loader,database_grasp,p_tmp.c_str(),success);
			/* debug start
			 counter +=1;
			 if(counter >= 0){
			 std::cout << " debug stop " << std::endl;
			 break;
			 }
			 // debug end
			 */
		}

	}
		data_storage.Store_database(database_grasp);
	} catch (const fs::filesystem_error &ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}
