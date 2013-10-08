/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         visualization.cpp

 \author       Alexander Herzog, Daniel Kappler
 \date         April 15, 2013

 *********************************************************************/

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <grasp_template/grasp_template.h>

namespace deep_learning {
class Visualization {
private:
	ros::NodeHandle _nh;

	ros::Publisher _pub_point_cloud;
	ros::Publisher _pub_object_cloud;
	ros::Publisher _pub_marker;
	ros::Publisher _pub_pose;
public:
	Visualization(ros::NodeHandle &nh);
	virtual ~Visualization() {
	}
	;

	bool Update_visualization(grasp_template::GraspTemplate &g_temp);
	bool Update_visualization(sensor_msgs::PointCloud2 &object_cloud);
	bool Update_points(sensor_msgs::PointCloud2 &object_cloud);
	bool Update_pose(geometry_msgs::Pose &view_point);
	bool Update_cube(geometry_msgs::Pose &pose, Eigen::Vector3d &dim);
};
}

#endif /* VISUALIZATION */
