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
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <deep_learning/utils.h>

namespace fs = boost::filesystem3;
namespace po = boost::program_options;
using namespace deep_learning;

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_data_grasp_database_yaml";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	std::string _dir_path_database;
	std::string _database_name;
	if (!nh.getParam("dir_path_database", _dir_path_database)) {
		ROS_ERROR("dir path database");
		return -1;
	}

	if (!nh.getParam("database_name", _database_name)) {
		ROS_ERROR("dir path dataset");
		return -1;
	}

	std::vector<Data_grasp> database_grasps;
	Data_loader data_loader;
	Data_storage data_storage;
	data_storage.Init_database(_dir_path_database, _database_name);
	data_loader.Load_grasp_database(
			data_storage.Get_file_path_database().c_str(),
			"/deep_learning_data_grasp_database", database_grasps);
	std::cout << " test grasps " << std::endl;


	YAML::Emitter doc;
	boost::hash<grasp_database_hash> data_grasp_hasher;
	for (unsigned int i = 0; i < database_grasps.size(); ++i) {
		std::cout << database_grasps[i].uuid_database << std::endl;
		std::cout << database_grasps[i].gripper_pose << std::endl;
		Data_grasp grasp = database_grasps[i];
		doc << YAML::BeginMap;
		doc << YAML::Key << grasp.uuid_database;
		doc << YAML::Value;
		doc << YAML::BeginMap;
		doc << YAML::Key << "gripper_pose" << YAML::Value << YAML::Flow
				<< YAML::BeginSeq
				<< grasp.gripper_pose.position.x
				<< grasp.gripper_pose.position.y
				<< grasp.gripper_pose.position.z
				<< grasp.gripper_pose.orientation.x
				<< grasp.gripper_pose.orientation.y
				<< grasp.gripper_pose.orientation.z
				<< YAML::EndSeq;
		doc << YAML::Key << "joints" << YAML::Value << YAML::Flow
				<< grasp.gripper_joints;


		doc << YAML::Key << "hash" << YAML::Value << data_grasp_hasher(grasp_database_hash(&grasp.gripper_joints,&grasp.gripper_pose));
		doc << YAML::EndMap;
		doc << YAML::EndMap;

	}

	std::string path = _dir_path_database + "/" + _database_name + ".yaml";
	std::ofstream fout(path.c_str());
	fout << doc.c_str();
	std::cout << doc.c_str();
	fout.close();
	return 0;
}
