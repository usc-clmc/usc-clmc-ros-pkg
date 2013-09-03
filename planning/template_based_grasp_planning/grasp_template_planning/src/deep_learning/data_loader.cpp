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
	_pextract_template = NULL;

	// arm
	//Eigen::Vector3d bounding_box_corner_1(-0.12, -0.12, -0.07);
	//Eigen::Vector3d bounding_box_corner_2(0.9, 0.12, 0.15);
	// pr2
	Eigen::Vector3d bounding_box_corner_1(-0.005, -0.12, -0.07);
	Eigen::Vector3d bounding_box_corner_2(0.16, 0.12, 0.02);
	_pextract_template = new Extract_template(bounding_box_corner_1,
			bounding_box_corner_2);
}

Database_grasp Data_loader::Load_grasp_database(const std::string &path_bagfile) {

	// read execution log from the bagfile
	std::vector<Data_grasp_log> database;
	try {
		ROS_DEBUG("load database grasp %s",path_bagfile.c_str());
		usc_utilities::FileIO<Data_grasp_log>::readFromBagFile(database, DATABASE_GRASP_TOPIC,
				path_bagfile);
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
		std::vector<Data_grasp> &result_data_grasps,int success) {
	grasp_template_planning::GraspDemoLibrary g_lib("", path_bagfile);
	g_lib.loadLibraryFull();

	for (std::vector<grasp_template_planning::GraspAnalysis,
			Eigen::aligned_allocator<grasp_template_planning::GraspAnalysis> >::const_iterator it =
			g_lib.getAnalysisMsgs()->begin();
			it != g_lib.getAnalysisMsgs()->end(); it++) {

		grasp_template::GraspTemplate g_temp = grasp_template::GraspTemplate(
				it->grasp_template, it->template_pose.pose);

		geometry_msgs::Pose gripper_pose_offset;
		Extract_template::Coordinate_to_base(it->template_pose.pose,
				it->gripper_pose.pose, gripper_pose_offset);

		grasp_template::DismatchMeasure d_measure(it->grasp_template,
				it->template_pose.pose, it->gripper_pose.pose,
				_pextract_template->_bounding_box_corner_1,
				_pextract_template->_bounding_box_corner_2);

		d_measure.applyDcMask(g_temp);


		result_data_grasps.push_back(
				Data_grasp(it->fingerpositions.vals,
						gripper_pose_offset, g_temp,success));
	}
	return true;
}


bool Data_loader::Load_supervised_from_library(const std::string &path_bagfile,
		std::vector<Dataset_grasp> &result_dataset_grasps) {
	return false;
	/*
	grasp_template_planning::GraspDemoLibrary g_lib("", path_bagfile);
	g_lib.loadLibraryFull();

	boost::hash<grasp_database_hash> data_grasp_hasher;
	for (std::vector<grasp_template_planning::GraspAnalysis,
			Eigen::aligned_allocator<grasp_template_planning::GraspAnalysis> >::const_iterator it =
			g_lib.getAnalysisMsgs()->begin();
			it != g_lib.getAnalysisMsgs()->end(); it++) {

		grasp_template::GraspTemplate g_temp = grasp_template::GraspTemplate(
				it->grasp_template, it->template_pose.pose);

		geometry_msgs::Pose gripper_pose_offset;
		Extract_template::Coordinate_to_base(it->template_pose.pose,
				it->gripper_pose.pose, gripper_pose_offset);

		grasp_template::DismatchMeasure d_measure(it->grasp_template,
				it->template_pose.pose, it->gripper_pose.pose,
				_pextract_template->_bounding_box_corner_1,
				_pextract_template->_bounding_box_corner_2);

		d_measure.applyDcMask(g_temp);

		result_dataset_grasps.push_back(
				Dataset_grasp(path_bagfile, g_temp, gripper_pose_offset,
						transform_success(it->grasp_success)));
	}
	return true;
	*/
}

bool Data_loader::Load_supervised_from_grasp_analysis(
		const std::string &path_bagfile, const std::string &topic,
		std::vector<Dataset_grasp> &result_dataset_grasps) {
	return false;

	/*
	// read execution log from the bagfile
	std::vector<grasp_template_planning::GraspAnalysis> grasp_analysis_log;
	try {
		std::cout << "load dataset supervised from grasp analysis" << std::endl;
		usc_utilities::FileIO<grasp_template_planning::GraspAnalysis>::readFromBagFile(
				grasp_analysis_log, topic, path_bagfile);
	} catch (rosbag::BagUnindexedException &e) {
		std::cout << "bag reading error" << e.what() << std::endl;
		return false;
	}
	std::cout << "grasp_analysis len " << grasp_analysis_log.size()
			<< std::endl;
	boost::hash<grasp_database_hash> database_hasher;
	for (unsigned int i = 0; i < grasp_analysis_log.size(); ++i) {
		grasp_template_planning::GraspAnalysis tmp = grasp_analysis_log[i];

		grasp_template::GraspTemplate g_temp = grasp_template::GraspTemplate(
				tmp.grasp_template, tmp.template_pose.pose);

		grasp_template::DismatchMeasure d_measure(tmp.grasp_template,
				tmp.template_pose.pose, tmp.gripper_pose.pose,
				_pextract_template->_bounding_box_corner_1,
				_pextract_template->_bounding_box_corner_2);

		d_measure.applyDcMask(g_temp);

		geometry_msgs::Pose gripper_pose_offset;
		Extract_template::Coordinate_to_base(tmp.template_pose.pose,
				tmp.gripper_pose.pose, gripper_pose_offset);

		result_dataset_grasps.push_back(
				Dataset_grasp(path_bagfile, g_temp, gripper_pose_offset,
						tmp.grasp_success));
	}
	return true;
	*/
}

bool Data_loader::Load_supervised_from_grasp_log(
		const std::string &path_bagfile, const std::string &topic,
		std::vector<Dataset_grasp> &result_dataset_grasps) {
	return false;

	/*
	// read execution log from the bagfile
	std::vector<grasp_template_planning::GraspLog> grasp_trial_log;
	try {
		std::cout << "load dataset supervised from grasp log" << std::endl;
		usc_utilities::FileIO<grasp_template_planning::GraspLog>::readFromBagFile(
				grasp_trial_log, topic, path_bagfile);
	} catch (rosbag::BagUnindexedException &e) {
		std::cout << "bag reading error" << e.what() << std::endl;
		return false;
	}

	if (grasp_trial_log.size() == 0) {
		return false;
	}

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
	grasp_template_planning::GraspAnalysis grasp_matched =
			grasp_trial_log_sorted[1]->matched_grasp;

	// grasp_success 0.5 undefined, 0.0 failed, 1.0 success
	if (grasp_applied.grasp_success < 0.4) {
		std::cout << " GRASP FAILED " << std::endl;
	}
	if (grasp_applied.grasp_success > 0.6) {
		std::cout << " GRASP SUCCESS " << std::endl;
	}

	if ((grasp_applied.grasp_success < 0.4)
			|| (grasp_applied.grasp_success > 0.6)) {
		grasp_template::GraspTemplate g_temp = grasp_template::GraspTemplate(
				grasp_applied.grasp_template, grasp_applied.template_pose.pose);

		grasp_template::DismatchMeasure d_measure(grasp_applied.grasp_template,
				grasp_applied.template_pose.pose,
				grasp_applied.gripper_pose.pose);
		d_measure.applyDcMask(g_temp);

		for (unsigned int i = 0; i < grasp_applied.fingerpositions.vals.size();
				++i) {
			assert(
					grasp_applied.fingerpositions.vals[i]
							== grasp_matched.fingerpositions.vals[i]);
		}

		geometry_msgs::Pose gripper_pose_offset;
		Extract_template::Coordinate_to_base(grasp_matched.template_pose.pose,
				grasp_matched.gripper_pose.pose, gripper_pose_offset);

		geometry_msgs::Pose gripper_pose_offset_applied;
		Extract_template::Coordinate_to_base(grasp_applied.template_pose.pose,
				grasp_applied.gripper_pose.pose, gripper_pose_offset_applied);

		result_dataset_grasps.push_back(
				Dataset_grasp(path_bagfile, g_temp, gripper_pose_offset_applied,
						transform_success(grasp_applied.grasp_success)));

	}
	return true;
	*/
}

/*
Data_grasp Data_loader::Load_grasp_database(const std::string &path_bagfile,
		const std::string &topic) {

	// read execution log from the bagfile
	std::vector<grasp_template_planning::GraspLog> grasp_trial_log;
	try {
		std::cout << "load grasp template" << std::endl;
		usc_utilities::FileIO<grasp_template_planning::GraspLog>::readFromBagFile(
				grasp_trial_log, topic, path_bagfile);
	} catch (rosbag::BagUnindexedException &e) {
		std::cout << "bag reading error" << e.what() << std::endl;
		return Data_grasp();
	}

	if (grasp_trial_log.size() == 0) {
		return Data_grasp();
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

	// matched grasps are the ones in the library
	grasp_template_planning::GraspAnalysis grasp_matched =
			grasp_trial_log_sorted[1]->matched_grasp;

	geometry_msgs::Pose gripper_pose_offset;
	Extract_template::Coordinate_to_base(grasp_matched.template_pose.pose,
			grasp_matched.gripper_pose.pose, gripper_pose_offset);

	grasp_template::GraspTemplate g_temp(grasp_matched.grasp_template,
			grasp_matched.template_pose.pose);

	grasp_template::DismatchMeasure d_measure(grasp_matched.grasp_template,
			grasp_matched.template_pose.pose, grasp_matched.gripper_pose.pose,
			_pextract_template->_bounding_box_corner_1,
			_pextract_template->_bounding_box_corner_2);
	d_measure.applyDcMask(g_temp);

	std::cout << "gripper offset " << gripper_pose_offset << std::endl;
	std::cout << "gripper joints " << grasp_matched.fingerpositions.vals.size()
			<< std::endl;
	Data_grasp result_grasp(grasp_matched.fingerpositions.vals,
			gripper_pose_offset, g_temp);

	boost::hash<grasp_database_hash> data_grasp_hasher;
	result_grasp.uuid_database = data_grasp_hasher(
			grasp_database_hash(&grasp_matched.fingerpositions.vals,
					&gripper_pose_offset));

	return result_grasp;
}
*/

Dataset_grasp Data_loader::Load_grasp_dataset(const std::string &path_bagfile,
		const std::string &topic) {

	return Dataset_grasp();
	/*
	// read execution log from the bagfile
	std::vector<grasp_template_planning::GraspLog> grasp_trial_log;
	try {
		std::cout << "load grasp template" << std::endl;
		usc_utilities::FileIO<grasp_template_planning::GraspLog>::readFromBagFile(
				grasp_trial_log, topic, path_bagfile);
	} catch (rosbag::BagUnindexedException &e) {
		std::cout << "bag reading error" << e.what() << std::endl;
		return Dataset_grasp();
	}

	if (grasp_trial_log.size() == 0) {
		return Dataset_grasp();
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

	// matched grasps are the ones in the library
	grasp_template_planning::GraspAnalysis grasp_matched =
			grasp_trial_log_sorted[1]->matched_grasp;

	grasp_template::GraspTemplate g_temp(grasp_matched.grasp_template,
			grasp_matched.template_pose.pose);

	grasp_template::DismatchMeasure d_measure(grasp_matched.grasp_template,
			grasp_matched.template_pose.pose, grasp_matched.gripper_pose.pose,
			_pextract_template->_bounding_box_corner_1,
			_pextract_template->_bounding_box_corner_2);
	d_measure.applyDcMask(g_temp);

	geometry_msgs::Pose gripper_pose_offset;
	Extract_template::Coordinate_to_base(grasp_matched.template_pose.pose,
			grasp_matched.gripper_pose.pose, gripper_pose_offset);

	boost::hash<grasp_database_hash> database_hasher;
	return Dataset_grasp(path_bagfile, g_temp, gripper_pose_offset,
			database_hasher(
					grasp_database_hash(&grasp_matched.fingerpositions.vals,
							&gripper_pose_offset)), SUCCESS_TRUE);
	*/
}

void Data_loader::Init_dataset(std::vector<Data_grasp> &grasp_templates) {
	_pextract_template->Init_gripper_offsets(grasp_templates);
	_grasp_templates.resize(grasp_templates.size());
	std::copy(grasp_templates.begin(), grasp_templates.end(),
			_grasp_templates.begin());
}

bool Data_loader::Load_unsupervised_from_grasp_log(
		const std::string &path_bagfile, const std::string &topic,
		std::vector<Dataset_grasp> &result_dataset_grasps) {
	return false;
	/*
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

	std::vector<std::size_t> uuids_data_grasp;
	std::vector<double> scores;
	uuids_data_grasp.resize(_grasp_templates.size());
	scores.resize(_grasp_templates.size());
	for (unsigned int k = 0; k < _grasp_templates.size(); ++k) {
		uuids_data_grasp[k] = _grasp_templates[k].uuid_database;
	}

	grasp_template::DismatchMeasure dissmatch_measure;
	grasp_template::TemplateDissimilarity score;
	// HsIterator is implemented in the same header where I implemented HeightmapSampling
	for (grasp_template::HsIterator it = heightmap_computation.getIterator();
			!it.passedLast() && ros::ok(); it.inc()) {

		grasp_template::GraspTemplate g_temp;
		// check the overloaded functions of generateTemplateOnHull and generateTemplate
		// they provide more options to extract templates
		heightmap_computation.generateTemplateOnHull(g_temp, it);

		std::vector<grasp_template::GraspTemplate,
				Eigen::aligned_allocator<grasp_template::GraspTemplate> > random_templates;
		std::vector<geometry_msgs::Pose> gripper_pose;

		_pextract_template->Get_random_grasp_templates(g_temp, random_templates,
				gripper_pose);

		for (unsigned int j = 0; j < random_templates.size(); ++j) {
			// score

			for (unsigned int k = 0; k < _grasp_templates.size(); ++k) {
				score = dissmatch_measure.getScore(
						_grasp_templates[k].grasp_template,
						random_templates[j]);
				scores[k] = score.getScore();
			}
			result_dataset_grasps.push_back(
					Dataset_grasp(path_bagfile, random_templates[j],
							gripper_pose[j], uuids_data_grasp, scores));
		}
//ROS_ERROR("finish for faster loading");
//break;
	}

	return true;
	*/
}

}
