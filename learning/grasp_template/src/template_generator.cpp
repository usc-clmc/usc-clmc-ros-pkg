/*
 * template_generator.cpp
 *
 *  Created on: Feb 11, 2011
 *      Author: herzog
 */
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <Eigen3/Eigen>

#include <grasp_template/template_generator.h>

using namespace std;
using namespace pcl;
using namespace Eigen3;
using namespace visualization_msgs;

namespace grasp_template
{

bool TemplateGenerator::calculateNormals(/*TODO: some container for normal, point of interest, etc. */)
{
	NormalEstimation<PointXYZ, Normal> estimator;
	estimator.setInputCloud(point_cloud_);
	estimator.setRadiusSearch(0.05);  //TODO: NEEDS TO BE EVALUATED
	boost::shared_ptr<PointCloud<Normal> > pc;
	pc.reset(new PointCloud<Normal>);
//	estimator.compute(*pc);
	/* DEBUG: USE ESTIMATOR.COMPUTE INSTEAD */
	for(unsigned int i = 0; i < point_cloud_->size(); i++)
	{
		Vector3d v(1, 1, 1);
		v.normalize();
		Normal n;
		n.normal[0] = v(0);
		n.normal[1] = v(1);
		n.normal[2] = v(2);
		pc->push_back(n);
	}
	/* DEBUG */

	normals_ = pc;

	return true;
}

void TemplateGenerator::generateTemplates()
{
	calculateNormals();
	PointCloud<Normal>::const_iterator normals_it = normals_->begin();
	for(PointCloud<PointXYZ>::const_iterator points_it =
			point_cloud_->begin(); points_it != point_cloud_->end() &&
			normals_it != normals_->end(); points_it++,
			normals_it++)
	{
		/* get transformation to heightmap frame */
		Translation3d translation(-points_it->x, -points_it->y, -points_it->z);
		Quaterniond rot_to_hm_frame;
		rot_to_hm_frame.setFromTwoVectors(Vector3d(normals_it->normal[0],
				normals_it->normal[1], normals_it->normal[2]), Vector3d(0, 0, 1) );

		/* create templates for several angles about z-achsis in heightmap
		 * frame
		 */
		const double delta_angle = M_PI / 4; // TODO: NEEDS TO BE EVALUATED
		for(double angle = 0; angle < M_PI; angle += delta_angle)
		{
			/* at rotation about z-achsis to transformation */
			Quaterniond rot_about_z(AngleAxisd(angle, Vector3d(0, 0, 1) ) );
			GraspTemplate templt;
			templt.heightmap_frame_transform_ = (rot_about_z * rot_to_hm_frame) * translation;

			/* choose points in range for heightmap and set heights */
			for(PointCloud<PointXYZ>::const_iterator it = point_cloud_->begin();
					it != point_cloud_->end(); it++)
			{
				/* transform to heightmap frame */
				Vector3d p(it->x, it->y, it->z);
				p = templt.heightmap_frame_transform_ * p;

				/* check if point is in range */
				if(std::abs(p(0)) <= templt.heightmap_.getMapLengthX() / 2.0 &&
						std::abs(p(1)) <= templt.heightmap_.getMapLengthY() / 2.0)
				{
//					ROS_INFO("In generator: p(0) = %f, p(1) = %f", abs(p(0)), abs(p(1)));

					/* is in range, so add to heightmap */
					templt.heightmap_.setGridTile(p(0), p(1), p(2));
				}
				else
				{
					continue;
				}
			}  //for each point

//			templt.heightmap_.interpolateEmptyTiles();
			templates_.push_back(templt);
		}  //for each template
	};  //for each normal
}

Marker TemplateGenerator::getVisualizationPointCloud() const
{
	Marker point_list;

	point_list.header.frame_id = "/grasp_template_generator_frame";
	point_list.header.stamp = ros::Time::now();

	point_list.ns = "templt_cluster_viz";
	point_list.id = 0;

	point_list.type = Marker::POINTS;
	point_list.action = Marker::ADD;

	point_list.pose.orientation.w = 1.0;
	point_list.scale.x = 0.005;
	point_list.scale.y = 0.005;
	point_list.color.r = 1.0;
	point_list.color.a = 1.0;

	for(PointCloud<PointXYZ>::const_iterator points_it = point_cloud_->begin();
			points_it != point_cloud_->end(); points_it++)
	{
		/* set point */
		geometry_msgs::Point p;
		p.x = points_it->x;
		p.y = points_it->y;
		p.z = points_it->z;

		point_list.points.push_back(p);
	}

	return point_list;
}

Marker TemplateGenerator::getVisualizationNormals() const
{
	Marker line_list;

	line_list.header.frame_id = "/grasp_template_generator_frame";
	line_list.header.stamp = ros::Time::now();

	line_list.ns = "templt_cluster_viz";
	line_list.id = 1;

	line_list.type = Marker::LINE_LIST;
	line_list.action = Marker::ADD;

	line_list.pose.orientation.w = 1.0;
	line_list.scale.x = 0.1;
	line_list.color.g = 1.0;
	line_list.color.a = 1.0;

	PointCloud<Normal>::const_iterator normals_it = normals_->begin();
	for(PointCloud<PointXYZ>::const_iterator points_it =
			point_cloud_->begin(); points_it != point_cloud_->end() &&
			normals_it != normals_->end(); points_it++,
			normals_it++)
	{
		/* from point */
		geometry_msgs::Point p;
		p.x = points_it->x;
		p.y = points_it->y;
		p.z = points_it->z;

		line_list.points.push_back(p);

		/* in normal direction */
		p.x += normals_it->normal[0];
		p.y += normals_it->normal[1];
		p.z += normals_it->normal[2];

		line_list.points.push_back(p);
	}

	return line_list;
}

}  //namespace
