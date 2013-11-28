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

#include <deep_learning/image_transformations.h>

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
#include <boost/functional/hash.hpp>
#include <fstream>
#include <rosbag/bag.h>
#include <grasp_template_planning/deep_learning_grasp_dataset.h>
#include <deep_learning/utils.h>

namespace deep_learning {
namespace fs = boost::filesystem3;

void Data_storage::Init_dataset(const std::string &path,
		const std::string& dataset_name, Extract_template *pextract_template) {
	_pextract_template = pextract_template;
	fs::path path_dir(path);
	if (!fs::is_directory(path_dir)) {
		std::cout << "path_dir " << path_dir.c_str() << std::endl;
		fs::create_directory(path_dir);
	}
	_dir_path_dataset = path_dir;
	_file_path_dataset = _dir_path_dataset / fs::path(dataset_name + ".yaml");
	if (fs::exists(_file_path_dataset)) {
		std::ifstream fin(_file_path_dataset.c_str());
		YAML::Parser parser(fin);
		YAML::Node tmp;

		while (parser.GetNextDocument(tmp)) {
			_doc << tmp;
		}
	}
	fs::path file_path_grasp_dataset = _dir_path_dataset
			/ fs::path(dataset_name + ".bag");
	try {
		if (fs::exists(file_path_grasp_dataset)) {
			_grasp_dataset.open(file_path_grasp_dataset.c_str(),
					rosbag::bagmode::Append);
		} else {
			_grasp_dataset.open(file_path_grasp_dataset.c_str(),
					rosbag::bagmode::Write);
		}
	} catch (rosbag::BagUnindexedException &e) {
		fs::path file_path_grasp_dataset = _dir_path_dataset
				/ fs::path(dataset_name + "_error.bag");
		_grasp_dataset.close();
		_grasp_dataset.open(file_path_grasp_dataset.c_str(),
				rosbag::bagmode::Write);
	}

}

void Data_storage::Update_dataset(
		std::map<std::size_t, Dataset_grasp> &result_grasps) {
	std::map<std::size_t, Dataset_grasp>::iterator iter;
	for (iter = result_grasps.begin(); iter != result_grasps.end(); ++iter) {
		Update_dataset(iter->second);
	}
}

void Data_storage::Update_dataset(const Dataset_grasp &dataset_grasp) {
	// todo dk switch to hash over heightmap
	std::size_t hash_heightmap = hash_value(
			dataset_grasp.grasp_template.heightmap_);
	std::stringstream ss;
	ss << hash_heightmap;
	std::string uuid_dataset = ss.str();

	unsigned int size =
				dataset_grasp.grasp_template.heightmap_.getNumTilesX();
	cv::Mat image_full = cv::Mat::zeros(size, size, CV_8UC1);
	cv::Mat image_map = cv::Mat::zeros(size, size, CV_8UC1);
	Render_image(
			dataset_grasp.grasp_template.heightmap_,
				_pextract_template->_bounding_box_corner_1.z(),
				_pextract_template->_bounding_box_corner_2.z(),
				image_full,image_map);
	unsigned int image_size_transformed = 20;
	double angle = 0.0;
	bool flip = false;
	Normalize_image(image_full,angle,flip);

	{
		fs::path path_result = _dir_path_dataset
				/ fs::path(uuid_dataset + "--solid.bmp");

		cv::Mat image_solid = cv::Mat::zeros(size, size, CV_8UC1);
		Render_image(image_full,image_map,image_solid,0);


		// group by alex demo
		if (dataset_grasp.uuid_original != "") {
			fs::path dir_path_dataset = _dir_path_dataset
					/ fs::path(dataset_grasp.uuid_original);
			if (!fs::is_directory(dir_path_dataset)) {
				fs::create_directories(dir_path_dataset);
			}
			fs::path path_result = dir_path_dataset
					/ fs::path(uuid_dataset + "--solid.bmp");
			cv::imwrite(path_result.c_str(), image_solid);
		} else {
			fs::path dir_path_dataset = _dir_path_dataset / fs::path("empty");
			if (!fs::is_directory(dir_path_dataset)) {
				fs::create_directories(dir_path_dataset);
			}
			fs::path path_result = dir_path_dataset
					/ fs::path(uuid_dataset + "--solid.bmp");
			cv::imwrite(path_result.c_str(), image_solid);
		}

		// group by hash
		std::stringstream ss_database;
		ss_database << dataset_grasp.uuid_database;
		fs::path dir_path_dataset = _dir_path_dataset
				/ fs::path(ss_database.str());
		if (!fs::is_directory(dir_path_dataset)) {
			fs::create_directories(dir_path_dataset);
		}

		path_result = dir_path_dataset / fs::path(uuid_dataset + "--solid.bmp");
		cv::imwrite(path_result.c_str(), image_solid);
		path_result = dir_path_dataset / fs::path(uuid_dataset + "--solid-normalized.bmp");

		cv::Mat image_solid_normalized = Transform_image(image_solid,angle,flip,image_size_transformed);
		cv::imwrite(path_result.c_str(), image_solid_normalized);
	}
	{
		fs::path path_result = _dir_path_dataset
				/ fs::path(uuid_dataset + "--4channel.bmp");

		cv::Mat image_4_channel = cv::Mat::zeros(size, size, CV_8UC4);
		Render_image(image_full,image_map,image_4_channel);
		image_4_channel = Transform_image(image_4_channel,angle,flip,image_size_transformed);

		//cv::imwrite(path_result.c_str(), result);

		grasp_template_planning::deep_learning_grasp_dataset dp;
		dp.uuid_dataset = uuid_dataset;
		dp.num_tiles_x = image_4_channel.rows;
		dp.num_tiles_y = image_4_channel.cols;
		dp.dim_channels = 4;
		dp.data.resize(image_4_channel.rows * image_4_channel.cols * dp.dim_channels);

		int pos = 0;
		for (int c = 0; c < dp.dim_channels; ++c) {
			for (int i = 0; i < image_4_channel.rows; i++) {
				for (int j = 0; j < image_4_channel.cols; j++) {
					dp.data[pos] = (uint8_t) image_4_channel.at<cv::Vec4i>(i, j)[c];
					pos += 1;
				}
			}
		}

		try {
			_grasp_dataset.write("grasp_dataset", ros::Time::now(), dp);
		} catch (rosbag::BagIOException ex) {
			ROS_DEBUG_STREAM(
					"Problem when writing log file " << _grasp_dataset.getFileName().c_str() << " : " << ex.what());

			return;
		}

	}
	/*
	 {
	 fs::path path_result = _dir_path_dataset / fs::path(uuid + "--fog.png");
	 cv::Mat result = Visualization::Render_image_fog(heightmap);
	 cv::imwrite(path_result.c_str(), result);
	 }
	 {
	 fs::path path_result = _dir_path_dataset / fs::path(uuid + "--table.png");
	 cv::Mat result = Visualization::Render_image_table(heightmap);
	 cv::imwrite(path_result.c_str(), result);
	 }
	 {
	 fs::path path_result = _dir_path_dataset / fs::path(uuid + "--dontcare.png");
	 cv::Mat result = Visualization::Render_image_dontcare(heightmap);
	 cv::imwrite(path_result.c_str(), result);
	 }
	 */

	_doc << YAML::BeginMap;
	_doc << YAML::Key << uuid_dataset;
	_doc << YAML::Value;
	_doc << YAML::BeginMap;
	_doc << YAML::Key << "uuid_database" << YAML::Value
			<< dataset_grasp.uuid_database;
	_doc << YAML::Key << "grasp_success" << YAML::Value
			<< dataset_grasp.success;
	_doc << YAML::Key << "uuids_dataset" << YAML::Value << YAML::Flow
			<< dataset_grasp.uuids_database;
	_doc << YAML::Key << "scores" << YAML::Value << YAML::Flow
			<< dataset_grasp.scores;

	_doc << YAML::Key << "grasp applied";
	_doc << YAML::Value;
	_doc << YAML::BeginMap;
	_doc << YAML::Key << "gripper_pose" << YAML::Value << YAML::Flow
			<< YAML::BeginSeq << dataset_grasp.gripper_pose.position.x
			<< dataset_grasp.gripper_pose.position.y
			<< dataset_grasp.gripper_pose.position.z
			<< dataset_grasp.gripper_pose.orientation.x
			<< dataset_grasp.gripper_pose.orientation.y
			<< dataset_grasp.gripper_pose.orientation.z << YAML::EndSeq;
	_doc << YAML::EndMap;
	_doc << YAML::EndMap;
	_doc << YAML::EndMap;
}

void Data_storage::Store_dataset() {
	std::ofstream fout(_file_path_dataset.c_str());
	fout << _doc.c_str();
	_grasp_dataset.close();
}

void Data_storage::Init_hdf5(io_hdf5::IO_hdf5Ptr io_hdf5){
	_io_hdf5 = io_hdf5;
}
void  Data_storage::Update_hdf5(std::vector<Dataset_grasp> &result_grasps){
	std::cout << "update hdf5" << std::endl;
}

void Data_storage::Init_database(const std::string &path,
		const std::string &database_name) {
	fs::path path_dir(path);
	if (!fs::is_directory(path_dir)) {
		fs::create_directory(path_dir);
	}
	_file_path_database = path_dir / fs::path(database_name + ".bag");
}

void Data_storage::Store_database(Database_grasp &database_grasp) {

	_database.open(_file_path_database.c_str(), rosbag::bagmode::Write);

	std::vector<Data_grasp> result_database_grasps;
	database_grasp.Get_grasps(result_database_grasps);
	std::cout << "///////////////////////////////////////////////////////////"
			<< std::endl;
	std::cout << "store database" << std::endl;
	std::cout << "///////////////////////////////////////////////////////////"
			<< std::endl;
	std::cout << std::endl;
	std::cout << "all grasps " << result_database_grasps.size() << std::endl;

	database_type::iterator iter;
	for (unsigned int i = 0; i < result_database_grasps.size(); ++i) {
		Data_grasp tmp_grasp = result_database_grasps[i];
		Data_grasp_log grasp_log;
		grasp_log.stamp = ros::Time::now();
		grasp_log.success = tmp_grasp.success;
		grasp_log.uuid_original = tmp_grasp.uuid_original;
		grasp_log.uuid_database = tmp_grasp.uuid_database;
		grasp_log.uuid_database_template = tmp_grasp.uuid_database_template;
		grasp_log.gripper_joints.vals = tmp_grasp.gripper_joints;
		grasp_log.gripper_pose.pose = tmp_grasp.gripper_pose;
		tmp_grasp.grasp_template.heightmap_.toHeightmapMsg(
				grasp_log.grasp_template_heightmap);
		tmp_grasp.grasp_template.getPose(grasp_log.grasp_template_pose.pose);
		try {
			_database.write(DATABASE_GRASP_TOPIC, ros::Time::now(), grasp_log);
		} catch (rosbag::BagIOException &ex) {
			ROS_DEBUG_STREAM(
					"Problem when writing log file " << _database.getFileName().c_str() << " : " << ex.what());

			return;
		}
	}
	_database.close();
}

}
