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

	Init_meta_data();
	Init_database();
}

void Data_storage::Init_meta_data() {
	file_path_meta = _path_dir / fs::path("meta.yaml");
	if (fs::exists(file_path_meta)) {
		std::ifstream fin(file_path_meta.c_str());
		YAML::Parser parser(fin);
		YAML::Node tmp;

		while (parser.GetNextDocument(tmp)) {
			_doc << tmp;
		}
	}
}

void Data_storage::Init_database() {
	file_path_database = _path_dir / fs::path("database.bag");
}

bool Data_storage::Store(grasp_template::TemplateHeightmap &heightmap) {
	std::string uuid = boost::lexical_cast<std::string>(
			boost::uuids::random_generator()());
	fs::path path_result = _path_dir / fs::path(uuid + ".png");
	cv::Mat result = Render_image(heightmap);
	cv::imwrite(path_result.c_str(), result);

	_doc << YAML::BeginMap;
	_doc << YAML::Key << uuid;
	_doc << YAML::Value;
	_doc << YAML::BeginMap;
	_doc << YAML::Key << "grasp_uuid" << YAML::Value << "__NONE__";
	// todo define for grasp success
	_doc << YAML::Key << "grasp_success" << YAML::Value << 0.0;
	_doc << YAML::EndMap;
	_doc << YAML::EndMap;

	return true;
}

bool Data_storage::Store(grasp_template::TemplateHeightmap &heightmap,
		const std::string &grasp_uuid, float grasp_success) {
	std::string uuid = boost::lexical_cast<std::string>(
			boost::uuids::random_generator()());
	fs::path path_result = _path_dir / fs::path(uuid + ".png");

	cv::Mat result = Render_image(heightmap);
	cv::imwrite(path_result.c_str(), result);

	_doc << YAML::BeginMap;
	_doc << YAML::Key << uuid;
	_doc << YAML::Value;
	_doc << YAML::BeginMap;
	_doc << YAML::Key << "grasp_uuid" << YAML::Value << grasp_uuid;
	_doc << YAML::Key << "grasp_success" << YAML::Value << grasp_success;
	_doc << YAML::EndMap;
	_doc << YAML::EndMap;
	return true;
}

void Data_storage::Store_meta() {
	fs::path path_yaml = _path_dir / fs::path("meta.yaml");
	std::ofstream fout(path_yaml.c_str());
	fout << _doc.c_str();
}

cv::Mat Data_storage::Render_image(
		grasp_template::TemplateHeightmap &heightmap) {
	// compute the diagonal and position the pixels in the center
	assert(heightmap.getNumTilesX() == heightmap.getNumTilesY());
	unsigned int size = (unsigned int) sqrt(
			pow(heightmap.getNumTilesX(), 2) * 2);
	unsigned int offset = (size - heightmap.getNumTilesX()) / 2;
	cv::Mat result = cv::Mat(size, size, CV_32FC4);

	for (int ix = 0; ix < heightmap.getNumTilesX(); ++ix) {
		for (int iy = 0; iy < heightmap.getNumTilesY(); ++iy) {

			Eigen::Vector3d eig_point;
			heightmap.gridToWorldCoordinates(ix, iy, eig_point.x(),
					eig_point.y());

			double raw = heightmap.getGridTileRaw(ix, iy);
			if (heightmap.isUnset(raw) || heightmap.isEmpty(raw)) {
				eig_point.z() = 0;
			} else {
				eig_point.z() = heightmap.getGridTile(eig_point.x(),
						eig_point.y());
			}
			// the minus is just such that one has positive values
			// the actual value is pretty low since it is given in meters
			float z = -static_cast<float>(eig_point.z());

			if (heightmap.isSolid(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[0] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[0] = 0;
			}
			if (heightmap.isFog(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[1] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[1] = 0;
			}

			if (heightmap.isDontCare(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[2] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[2] = 0;
			}

			if (heightmap.isTable(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[3] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[3] = 0;
			}
		}
	}
	return result;
}

bool Data_storage::Store_grasp_database(
		std::map<std::string, Data_grasp> &result_grasps) {

	_database.open(file_path_database.c_str(), rosbag::bagmode::Write);
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

			return false;
		}

	}
	_database.close();
	return true;
}

}
