/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         data_storage.cpp

 \author       Daniel Kappler
 \date         July 31, 2013

 *********************************************************************/

#include <deep_learning/data_storage.h>

#include <deep_learning/visualization.h>

#include <assert.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <usc_utilities/file_io.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <fstream>

namespace deep_learning {
namespace fs = boost::filesystem3;

Data_storage::Data_storage(const std::string &path) {
	_path_dir = fs::path(path);
	if (!fs::is_directory(_path_dir)) {
		fs::create_directory(_path_dir);
	}
}

void Data_storage::Init_dataset(const std::string& dataset_name) {
	_file_path_dataset = _path_dir / fs::path(dataset_name + ".yaml");
	if (fs::exists(_file_path_dataset)) {
		std::ifstream fin(_file_path_dataset.c_str());
		YAML::Parser parser(fin);
		YAML::Node tmp;

		while (parser.GetNextDocument(tmp)) {
			_doc << tmp;
		}
	}
}

void Data_storage::Update_dataset(
		grasp_template::TemplateHeightmap &heightmap) {
	Update_dataset(heightmap, UUID_NONE, SUCCESS_FALSE);
}

void Data_storage::Update_dataset(grasp_template::TemplateHeightmap &heightmap,
		const std::string &grasp_uuid, float grasp_success) {
	std::string uuid = boost::lexical_cast<std::string>(
			boost::uuids::random_generator()());
	{
		fs::path path_result = _path_dir / fs::path(uuid + "--4channel.png");
		cv::Mat result = Visualization::Render_image_4channel(heightmap);
		cv::imwrite(path_result.c_str(), result);
	}
	{
		fs::path path_result = _path_dir / fs::path(uuid + "--solid.png");
		cv::Mat result = Visualization::Render_image_solid(heightmap);
		cv::imwrite(path_result.c_str(), result);
	}
	{
		fs::path path_result = _path_dir / fs::path(uuid + "--fog.png");
		cv::Mat result = Visualization::Render_image_fog(heightmap);
		cv::imwrite(path_result.c_str(), result);
	}
	{
		fs::path path_result = _path_dir / fs::path(uuid + "--table.png");
		cv::Mat result = Visualization::Render_image_table(heightmap);
		cv::imwrite(path_result.c_str(), result);
	}
	{
		fs::path path_result = _path_dir / fs::path(uuid + "--dontcare.png");
		cv::Mat result = Visualization::Render_image_dontcare(heightmap);
		cv::imwrite(path_result.c_str(), result);
	}

	_doc << YAML::BeginMap;
	_doc << YAML::Key << uuid;
	_doc << YAML::Value;
	_doc << YAML::BeginMap;
	_doc << YAML::Key << "grasp_uuid" << YAML::Value << grasp_uuid;
	_doc << YAML::Key << "grasp_success" << YAML::Value << grasp_success;
	_doc << YAML::EndMap;
	_doc << YAML::EndMap;
}

void Data_storage::Store_dataset() {
	std::ofstream fout(_file_path_dataset.c_str());
	fout << _doc.c_str();
}

void Data_storage::Init_database(const std::string &database_name) {
	_file_path_database = _path_dir / fs::path(database_name + ".bag");
}

void Data_storage::Store_database(
		std::map<std::string, Data_grasp> &result_grasps) {

	_database.open(_file_path_database.c_str(), rosbag::bagmode::Write);
	std::map<std::string, Data_grasp>::iterator iter;
	for (iter = result_grasps.begin(); iter != result_grasps.end(); ++iter) {
		std::cout << iter->first << std::endl;
		Data_grasp_log grasp_log;

		grasp_log.stamp = ros::Time::now();
		grasp_log.uuid = iter->second.uuid;
		grasp_log.gripper_pose.pose = iter->second.gripper_pose;
		std::cout << grasp_log.gripper_pose.pose << std::endl;
		iter->second.grasp_template.heightmap_.toHeightmapMsg(
				grasp_log.grasp_template_heightmap);
		iter->second.grasp_template.getPose(grasp_log.grasp_template_pose.pose);
		try {
			_database.write("/deep_learning_data_grasp_database",
					ros::Time::now(), grasp_log);
		} catch (rosbag::BagIOException &ex) {
			ROS_DEBUG_STREAM(
					"Problem when writing log file " << _database.getFileName().c_str() << " : " << ex.what());

			return;
		}

	}
	_database.close();
}

}
