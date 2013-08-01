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
#include <usc_utilities/file_io.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

Visualization::Visualization(ros::NodeHandle &nh) {
	_nh = nh;
	_pub_marker = _nh.advertise<visualization_msgs::Marker>("marker", 10);
}

bool Visualization::Update_visualization(
		grasp_template::GraspTemplate &g_temp) {
	// there are no frames available except for the base one so just use that to base the visualization on

	std::cout << "we are here" << std::endl;
	std::vector<visualization_msgs::Marker> v_hm = g_temp.getVisualization(
			"ns_name", "BASE");
	for (unsigned int i = 0; i < v_hm.size(); ++i) {
		_pub_marker.publish(v_hm[i]);
	}
	return true;
}

bool Visualization::Render_image(grasp_template::TemplateHeightmap &heightmap) {
	cv::Mat solid(heightmap.getNumTilesX(), heightmap.getNumTilesY(), CV_32FC1);
	cv::Mat fog(heightmap.getNumTilesX(), heightmap.getNumTilesY(), CV_32FC1);
	cv::Mat table(heightmap.getNumTilesX(), heightmap.getNumTilesY(), CV_32FC1);
	cv::Mat dont_care(heightmap.getNumTilesX(), heightmap.getNumTilesY(),
			CV_32FC1);

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

			if (heightmap.isEmpty(raw)) {
			} else {

			}
			if (heightmap.isSolid(raw)) {
				solid.at<float>(ix, iy) = z;
			} else {
				solid.at<float>(ix, iy) = 0;
			}
			if (heightmap.isFog(raw)) {
				fog.at<float>(ix, iy) = z;
			} else {
				fog.at<float>(ix, iy) = 0;
			}

			if (heightmap.isDontCare(raw)) {
				dont_care.at<float>(ix, iy) = z;
			} else {
				dont_care.at<float>(ix, iy) = 0;
			}

			if (heightmap.isTable(raw)) {
				table.at<float>(ix, iy) = z;
			} else {
				table.at<float>(ix, iy) = 0;
			}
		}
	}
	std::cout << "solid \n" << solid << std::endl;
	cv::imwrite("/tmp/solid.jpg", solid);
	cv::imwrite("/tmp/fog.jpg", fog);
	cv::imwrite("/tmp/table.jpg", table);
	cv::imwrite("/tmp/dont_care.jpg", dont_care);
	return true;
}
