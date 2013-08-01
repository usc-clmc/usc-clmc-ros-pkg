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

#include <deep_learning/log_loader.h>

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

#include <grasp_template/Heightmap.h>
#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

Log_loader::Log_loader(const std::string &log_topic) {
	_log_topic = log_topic;
}

bool Log_loader::Load_trial_log(const std::string &path_bagfile,
		std::vector<grasp_template::GraspTemplate,
				Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
		std::vector<std::string> &result_uuid,
		std::vector<float> &result_success) {

	// read execution log from the bagfile
	std::vector<grasp_template_planning::GraspLog> grasp_trial_log;
	usc_utilities::FileIO<grasp_template_planning::GraspLog>::readFromBagFile(
			grasp_trial_log, _log_topic, path_bagfile);

	// this is disgusting, but the content of GraspLog is spread over 3 messages
	// in the following we sort the messages
	std::vector<const grasp_template_planning::GraspLog*> grasp_trial_log_sorted;
	grasp_trial_log_sorted.resize(3);
	for (unsigned int i = 0; i < grasp_trial_log.size(); i++) {
		const int seq_nr = grasp_trial_log[i].seq;
		grasp_trial_log_sorted[seq_nr] = &(grasp_trial_log[i]);
	}

	// here we pull out what is required for heightmap computation
	sensor_msgs::PointCloud2 object_cloud =
			grasp_trial_log_sorted[0]->target_object;
	geometry_msgs::Pose table_frame = grasp_trial_log_sorted[1]->table_frame;
	geometry_msgs::Point vp =
			grasp_trial_log_sorted[1]->applied_grasp.viewpoint_transform.pose.position;
	geometry_msgs::Quaternion vo =
			grasp_trial_log_sorted[1]->applied_grasp.viewpoint_transform.pose.orientation;
	grasp_template_planning::GraspAnalysis grasp_analysis =
			grasp_trial_log_sorted[1]->matched_grasp;
	grasp_template::Heightmap grasp_heightmap = grasp_analysis.grasp_template;
	geometry_msgs::Pose template_pose = grasp_analysis.template_pose.pose;
	geometry_msgs::Pose gripper_pose = grasp_analysis.gripper_pose.pose;

	// convert some data structures
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(object_cloud, pcl_cloud);

	Eigen::Vector3d viewpoint_pos(vp.x, vp.y, vp.z);
	Eigen::Quaterniond viewpoint_rot(vo.w, vo.x, vo.y, vo.z);

	// this guy generates heightmaps from point clouds
	grasp_template::HeightmapSampling heightmap_computation = grasp_template::HeightmapSampling(viewpoint_pos,
			viewpoint_rot);
	heightmap_computation.initialize(pcl_cloud, table_frame);

	// reset all result vectors
	result_template.clear();
	result_uuid.clear();
	result_success.clear();

	// HsIterator is implemented in the same header where I implemented HeightmapSampling
	for (grasp_template::HsIterator it = heightmap_computation.getIterator();
			!it.passedLast() && ros::ok(); it.inc()) {
		grasp_template::GraspTemplate t;

		// check the overloaded functions of generateTemplateOnHull and generateTemplate
		// they provide more options to extract templates
		heightmap_computation.generateTemplateOnHull(t, it);
		grasp_template::DismatchMeasure d_measure(grasp_analysis.grasp_template,
				grasp_analysis.template_pose.pose,
				grasp_analysis.gripper_pose.pose);
		d_measure.applyDcMask(t);
		result_template.push_back(t);
		result_uuid.push_back("__NONE__");
		result_success.push_back(-1.0);
		ROS_ERROR("finish for faster loading");
		break;
	}
	return true;
}
