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
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h> // for PointXYZ
#include <ros/ros.h>
#include <map>
#include <boost/filesystem.hpp>
#include <io_hdf5/io_hdf5.h>

#include <iostream>

namespace fs = boost::filesystem3;
using namespace deep_learning;

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "main_dataset_from_point_clouds";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);

	std::string _dir_path_base;
	std::string _dir_path_database;
	std::string _file_path_point_clouds;
	std::string _dataset_name;
	std::string _database_name;

	if (!nh.getParam("dir_path_base", _dir_path_base)) {
		ROS_ERROR("dir path base");
		return -1;
	}
	if (!nh.getParam("dir_path_database", _dir_path_database)) {
		ROS_ERROR("dir path base");
		return -1;
	}
	if (!nh.getParam("file_path_point_clouds", _file_path_point_clouds)) {
		ROS_ERROR("file path point clouds");
		return -1;
	}
	if (!nh.getParam("dataset_name", _dataset_name)) {
		ROS_ERROR("dataset name");
		return -1;
	}
	if (!nh.getParam("database_name", _database_name)) {
		ROS_ERROR("database name");
		return -1;
	}

	Visualization visualization(nh);
	try {
		fs::path dir_path_base(_dir_path_base);
		Data_storage data_storage;
		Data_loader data_loader;
		fs::path dir_path_database = dir_path_base
				/ fs::path(_dir_path_database);
		Extract_template extract_template(BOUNDING_BOX_CORNER_1,
				BOUNDING_BOX_CORNER_2);

		io_hdf5::IO_hdf5Ptr io_hdf5(new io_hdf5::IO_hdf5(_file_path_point_clouds,
				H5F_ACC_RDWR));

		data_storage.Init_hdf5(io_hdf5);

		H5::Group group_point_cloud = io_hdf5->Get_group("point_cloud");
		std::vector<std::string> dataset_names;
		io_hdf5->Get_dataset_names(group_point_cloud, dataset_names);

		for (unsigned int i = 0; i < dataset_names.size(); ++i) {
			std::cout << "load dataset " << dataset_names[i] << std::endl;

			std::vector<Dataset_grasp> result_dataset_grasp;
			Eigen::Vector3d viewpoint_pos(0, 0, 0.7);
			Eigen::Quaterniond viewpoint_rot;
			pcl::PointCloud<pcl::PointXYZ> point_cloud;
			io_hdf5->Get_dataset(group_point_cloud, dataset_names[i],
					point_cloud);

			// debugging
			sensor_msgs::PointCloud2 cloud2;
			pcl::toROSMsg(point_cloud, cloud2);
			visualization.Update_points(cloud2);
			visualization.Update_visualization(cloud2);
			// debugging

			data_loader.Process_point_cloud(dataset_names[i], point_cloud,
					viewpoint_pos, viewpoint_rot, extract_template,
					result_dataset_grasp);
			data_storage.Update_hdf5(result_dataset_grasp);
		}
		io_hdf5->Close();

	} catch (const fs::filesystem_error &ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}
