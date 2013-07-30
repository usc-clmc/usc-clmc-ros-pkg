/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         howto_extract_templates.cpp

 \author       Alexander Herzog
 \date         April 15, 2013

 *********************************************************************/

#include <visualization.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <usc_utilities/file_io.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

Visualization::Visualization(ros::NodeHandle &nh) {
	_nh = nh;
}
grasp_template::HsIterator Visualization::Initilization() {
	// define path to the bagfile and topic names of messages
	// that we want to pull out from the bag file
	// should be defined in the launch file such that one does not have to recompile so often
	std::string BAGFILE_NAME;

	if (!_nh.getParam("bagfile_name", BAGFILE_NAME)) {
		std::cout << "could not get the parameter " << _nh.getNamespace() << "/bagfile_name" << std::endl;
		return grasp_template::HsIterator(0);
	}
	std::cout << "bagfile_name " << BAGFILE_NAME << std::endl;
	const std::string SNAPSHOT_TOPIC = "/grasp_planning_image_object";
	const std::string LOG_TOPIC = "/grasp_planning_log";

	// read snapshot of the object from the bagfile
	sensor_msgs::Image table_snapshot;
    usc_utilities::FileIO<sensor_msgs::Image>::readFromBagFile(table_snapshot, SNAPSHOT_TOPIC, BAGFILE_NAME);

	// read execution log from the bagfile
	std::vector<grasp_template_planning::GraspLog> grasp_trial_log;
	usc_utilities::FileIO<grasp_template_planning::GraspLog>::readFromBagFile(grasp_trial_log, LOG_TOPIC, BAGFILE_NAME);

	// this is disgusting, but the content of GraspLog is spread over 3 messages
	// in the following we sort the messages
	std::vector<const grasp_template_planning::GraspLog*> grasp_trial_log_sorted;
	grasp_trial_log_sorted.resize(3);
	for (unsigned int i = 0; i < grasp_trial_log.size(); i++) {
		std::cout << "read log message with seq nr " << grasp_trial_log[i].seq
				<< std::endl;
		const int seq_nr = grasp_trial_log[i].seq;
		grasp_trial_log_sorted[seq_nr] = &(grasp_trial_log[i]);

	}

	// here we pull out what is required for heightmap computation
	_object_cloud = grasp_trial_log_sorted[0]->target_object;
	geometry_msgs::Pose table_frame = grasp_trial_log_sorted[1]->table_frame;
	geometry_msgs::Point vp =
			grasp_trial_log_sorted[1]->applied_grasp.viewpoint_transform.pose.position;
	geometry_msgs::Quaternion vo =
			grasp_trial_log_sorted[1]->applied_grasp.viewpoint_transform.pose.orientation;
	grasp_template_planning::GraspAnalysis grasp_analysis = grasp_trial_log_sorted[1]->matched_grasp;
	_grasp_heightmap = grasp_analysis.grasp_template;
	_template_pose = grasp_analysis.template_pose.pose;
	_gripper_pose = grasp_analysis.gripper_pose.pose;

	_pub_point_cloud = _nh.advertise < sensor_msgs::PointCloud2
			> ("object_point_cloud", 1);

	// convert some data structures
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(_object_cloud, pcl_cloud);

	Eigen::Vector3d viewpoint_pos(vp.x, vp.y, vp.z);
	Eigen::Quaterniond viewpoint_rot(vo.w, vo.x, vo.y, vo.z);

	// this guy generates heightmaps from point clouds
	_heightmap_computation = grasp_template::HeightmapSampling(viewpoint_pos, viewpoint_rot);
	_heightmap_computation.initialize(pcl_cloud, table_frame);

	_pub_marker = _nh.advertise < visualization_msgs::Marker > ("marker", 1);
	return _heightmap_computation.getIterator();
}

bool Visualization::Update_visualization(grasp_template::HsIterator &hs_iter) {
	if (!hs_iter.passedLast()) {
		grasp_template::GraspTemplate t;

		// check the overloaded functions of generateTemplateOnHull and generateTemplate
		// they provide more options to extract templates
		_heightmap_computation.generateTemplateOnHull(t, hs_iter);
		grasp_template::DismatchMeasure d_measure(_grasp_heightmap,
				_template_pose, _gripper_pose);

		d_measure.applyDcMask(t);

		// fix that should be pr2 base or arm base have to look that up in the template config armrobot
		std::vector<visualization_msgs::Marker> v_hm = t.getVisualization(
				"ns_name", "BASE");
		_pub_point_cloud.publish(_object_cloud);
		for (unsigned int i = 0; i < v_hm.size(); ++i) {
			_pub_marker.publish(v_hm[i]);
		}

		hs_iter.inc();
		return true;
	}
	return false;
}
