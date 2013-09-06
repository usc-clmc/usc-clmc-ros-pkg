/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         data_loader.cpp

 \author       Daniel Kappler
 \date         July 31, 2013

 *********************************************************************/

#include <deep_learning/data_loader.h>

#include <deep_learning/def.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <usc_utilities/file_io.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/Heightmap.h>
#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>
#include <grasp_template_planning/template_matching.h>
#include <grasp_template_planning/grasp_demo_library.h>
#include <cmath>
#include <deep_learning/utils.h>

namespace deep_learning {
Data_loader::Data_loader() {
}

Database_grasp Data_loader::Load_grasp_database(
		const std::string &path_bagfile) {

	// read execution log from the bagfile
	std::vector<Data_grasp_log> database;
	try {
		ROS_DEBUG("load database grasp %s", path_bagfile.c_str());
		usc_utilities::FileIO<Data_grasp_log>::readFromBagFile(database,
				DATABASE_GRASP_TOPIC, path_bagfile);
	} catch (rosbag::BagUnindexedException &e) {
		std::cout << "bag reading error" << e.what() << std::endl;
		return Database_grasp();
	}

	// reset all result vectors
	Database_grasp result_database_grasp;

	for (unsigned int i = 0; i < database.size(); ++i) {
		Data_grasp tmp_grasp(database[i]);
		result_database_grasp.Add_grasp(tmp_grasp);
	}
	return result_database_grasp;
}

bool Data_loader::Load_from_library(const std::string &path_bagfile,
		std::vector<Data_grasp> &result_data_grasps, int success) {
	grasp_template_planning::GraspDemoLibrary g_lib("", path_bagfile);
	g_lib.loadLibraryFull();

	for (std::vector<grasp_template_planning::GraspAnalysis,
			Eigen::aligned_allocator<grasp_template_planning::GraspAnalysis> >::const_iterator it =
			g_lib.getAnalysisMsgs()->begin();
			it != g_lib.getAnalysisMsgs()->end(); it++) {
		grasp_template_planning::GraspAnalysis tmp(*it);
		Data_grasp::Add(tmp, success, result_data_grasps);
	}
	return true;
}

bool Data_loader::Load_from_grasp_log(const std::string &path_bagfile,
		const std::string &topic, Extract_template &extract_template,
		std::vector<Dataset_grasp> &result_dataset_grasps) {
	// reset all result vectors
	result_dataset_grasps.clear();

	// read execution log from the bagfile
	std::vector<grasp_template_planning::GraspLog> grasp_trial_log;
	try {
		std::cout << "load dataset unsupervised from grasp log" << std::endl;
		usc_utilities::FileIO<grasp_template_planning::GraspLog>::readFromBagFile(
				grasp_trial_log, topic, path_bagfile);
	} catch (rosbag::BagUnindexedException &e) {
		std::cout << "bag reading error" << e.what() << std::endl;
		return false;
	}

	if (grasp_trial_log.size() == 0) {
		std::cout << "bagfile was empty" << std::endl;
		return false;
	}

	// grasp_trial_log is sorted because in the bag they might not
	// grasp_trial_log is split up such that the messages are smaller

	// this is disgusting, but the content of GraspLog is spread over 3 messages
	// in the following we sort the messages
	std::vector<const grasp_template_planning::GraspLog*> grasp_trial_log_sorted;
	grasp_trial_log_sorted.resize(3);
	for (unsigned int i = 0; i < grasp_trial_log.size(); i++) {
		const int seq_nr = grasp_trial_log[i].seq;
		grasp_trial_log_sorted[seq_nr] = &(grasp_trial_log[i]);

	}

	// applied grasp is the one executed
	grasp_template_planning::GraspAnalysis grasp_applied =
			grasp_trial_log_sorted[1]->applied_grasp;

	// here we pull out what is required for heightmap computation
	sensor_msgs::PointCloud2 object_cloud =
			grasp_trial_log_sorted[0]->target_object;

	geometry_msgs::Pose table_frame = grasp_trial_log_sorted[1]->table_frame;
	geometry_msgs::Point vp = grasp_applied.viewpoint_transform.pose.position;
	geometry_msgs::Quaternion vo =
			grasp_applied.viewpoint_transform.pose.orientation;

	// matched grasps are the ones in the library
	grasp_template_planning::GraspAnalysis grasp_matched =
			grasp_trial_log_sorted[1]->matched_grasp;

	grasp_template::Heightmap grasp_heightmap = grasp_matched.grasp_template;
	geometry_msgs::Pose template_pose = grasp_matched.template_pose.pose;
	geometry_msgs::Pose gripper_pose = grasp_matched.gripper_pose.pose;

	// convert some data structures
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(object_cloud, pcl_cloud);

	Eigen::Vector3d viewpoint_pos(vp.x, vp.y, vp.z);
	Eigen::Quaterniond viewpoint_rot(vo.w, vo.x, vo.y, vo.z);

	// this guy generates heightmaps from point clouds
	grasp_template::HeightmapSampling heightmap_computation =
			grasp_template::HeightmapSampling(viewpoint_pos, viewpoint_rot);
	heightmap_computation.initialize(pcl_cloud, table_frame);

	// HsIterator is implemented in the same header where I implemented HeightmapSampling
	for (grasp_template::HsIterator it = heightmap_computation.getIterator();
			!it.passedLast() && ros::ok(); it.inc()) {

		grasp_template::GraspTemplate g_temp;
		// check the overloaded functions of generateTemplateOnHull and generateTemplate
		// they provide more options to extract templates
		heightmap_computation.generateTemplateOnHull(g_temp, it);

		Dataset_grasp::Add(path_bagfile, g_temp, extract_template,
				result_dataset_grasps);

		//ROS_ERROR("finish for faster loading");
		//break;
	}

	return true;
}

}
