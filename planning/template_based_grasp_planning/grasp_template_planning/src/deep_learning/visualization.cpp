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

#include <deep_learning/visualization.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <usc_utilities/file_io.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

namespace deep_learning {

Visualization::Visualization(ros::NodeHandle &nh) {
	_nh = nh;
	_pub_marker = _nh.advertise<visualization_msgs::Marker>("marker", 10);
	_pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
	_pub_object_cloud = _nh.advertise<sensor_msgs::PointCloud2>("object_cloud",
			10);
	_pub_point_cloud = _nh.advertise<sensor_msgs::PointCloud2>("point_cloud",
			10);
}

bool Visualization::Update_visualization(
		grasp_template::GraspTemplate &g_temp) {
	// there are no frames available except for the base one so just use that to base the visualization on

	std::vector<visualization_msgs::Marker> v_hm = g_temp.getVisualizationBase(
			"ns_name", "BASE");
	for (unsigned int i = 0; i < v_hm.size(); ++i) {
		_pub_marker.publish(v_hm[i]);
	}
	return true;
}

bool Visualization::Update_visualization(
		sensor_msgs::PointCloud2 &object_cloud) {
	_pub_object_cloud.publish(object_cloud);
	return true;
}
bool Visualization::Update_points(sensor_msgs::PointCloud2 &point_cloud) {
	std::cout << "update points " << std::endl;
	point_cloud.header.frame_id = "BASE";
	point_cloud.header.stamp = ros::Time::now();
	_pub_point_cloud.publish(point_cloud);
	return true;
}

bool Visualization::Update_cube(geometry_msgs::Pose &pose,
		Eigen::Vector3d &dim) {
	visualization_msgs::Marker m_box;
	m_box.header.frame_id = "BASE";
	m_box.header.stamp = ros::Time::now();
	m_box.ns = "ns_box";
	m_box.id = 10;
	m_box.type = visualization_msgs::Marker::CUBE;
	m_box.action = visualization_msgs::Marker::ADD;
	m_box.scale.x = dim.x();
	m_box.scale.y = dim.y();
	m_box.scale.z = dim.z();
	m_box.color.r = 0.00;
	m_box.color.b = 1.00;
	m_box.color.g = 1.00;
	m_box.color.a = 0.20;

	m_box.pose.position.x = pose.position.x;
	m_box.pose.position.y = pose.position.y;
	m_box.pose.position.z = pose.position.z;
	m_box.pose.orientation.x = pose.orientation.x;
	m_box.pose.orientation.y = pose.orientation.y;
	m_box.pose.orientation.z = pose.orientation.z;
	m_box.pose.orientation.w = pose.orientation.w;

	_pub_marker.publish(m_box);

	return true;
}

bool Visualization::Update_pose(geometry_msgs::Pose &pose) {
	geometry_msgs::PoseStamped m_pose;
	m_pose.header.frame_id = "BASE";
	m_pose.header.stamp = ros::Time::now();
	m_pose.pose = pose;
	_pub_pose.publish(m_pose);
	return true;
}

}
