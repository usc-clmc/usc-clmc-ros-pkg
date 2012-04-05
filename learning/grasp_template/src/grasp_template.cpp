/*
 * grasp_template.cpp
 *
 *  Created on: Feb 14, 2011
 *      Author: herzog
 */

#include <vector>

#include <visualization_msgs/Marker.h>

#include <grasp_template/grasp_template.h>

using namespace visualization_msgs;
using namespace std;

namespace grasp_template
{

Marker GraspTemplate::getVisualization() const
{
	Marker line_list;

	line_list.header.frame_id = "/grasp_template_frame";
	line_list.header.stamp = ros::Time::now();

	line_list.ns = "templates_viz";
	line_list.id = 0;// iy * heightmap_.getNumTilesX() + ix;	//TODO: maybe define sth that makes more sense

	line_list.type = Marker::LINE_LIST;
	line_list.action = Marker::ADD;

	line_list.pose.orientation.w = 1.0;
	line_list.scale.x = 0.1;
	line_list.color.b = 1.0;
	line_list.color.a = 1.0;

	for(unsigned int ix = 0; ix < heightmap_.getNumTilesX(); ++ix)
	{
		for(unsigned int iy = 0; iy < heightmap_.getNumTilesY(); ++iy)
		{
			double p_x, p_y;
			heightmap_.gridToWorldCoordinates(ix, iy, p_x, p_y);
			geometry_msgs::Point p;

			/* from ground */
			p.x = static_cast<float>(p_x);
			p.y = static_cast<float>(p_y);
			p.z = 0;
			line_list.points.push_back(p);

			/* to height-value */
			p.z = static_cast<float>(heightmap_.getGridTile(ix, iy));
			line_list.points.push_back(p);
		}
	}

	return line_list;
}

}  //namespace
