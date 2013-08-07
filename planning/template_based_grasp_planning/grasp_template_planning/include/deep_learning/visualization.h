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

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <grasp_template/Heightmap.h>
#include <grasp_template/template_heightmap.h>
#include <grasp_template/heightmap_sampling.h>

class Visualization{
private:
	ros::NodeHandle _nh;

	ros::Publisher _pub_object_cloud;
	ros::Publisher _pub_marker;
	ros::Publisher _pub_view_point;
public:
	Visualization(ros::NodeHandle &nh);
	virtual ~Visualization(){};

	bool Update_visualization(grasp_template::GraspTemplate &g_temp);
	bool Update_visualization(sensor_msgs::PointCloud2 &object_cloud);
	bool Update_visualization(geometry_msgs::Pose &view_point);
	bool Render_image(grasp_template::TemplateHeightmap &g_temp);
};

#endif /* VISUALIZATION */
