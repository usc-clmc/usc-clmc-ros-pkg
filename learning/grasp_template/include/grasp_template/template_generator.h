/*
 * template_generator.h
 *
 *  Created on: Feb 11, 2011
 *      Author: herzog
 */

#ifndef TEMPLATE_GENERATOR_H_
#define TEMPLATE_GENERATOR_H_

#include <vector>
#include <boost/shared_ptr.hpp>

#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "template_heightmap.h"
#include "grasp_template.h"

namespace grasp_template
{

class TemplateGenerator
{
public:

	/*
	 * set point-cloud which describes an object
	 */
	void setPointCloud(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > data);

	/*
	 * generates templates
	 */
	void generateTemplates();

	const std::vector<GraspTemplate>& getTemplates() const{return templates_;};

	visualization_msgs::Marker getVisualizationNormals() const;
	visualization_msgs::Marker getVisualizationPointCloud() const;

private:

	boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > point_cloud_;
	boost::shared_ptr<const pcl::PointCloud<pcl::Normal> > normals_;
	std::vector<GraspTemplate> templates_;

	/*
	 * calculate normal vector at a region
	 * @return success
	 */
	bool calculateNormals();

};


void TemplateGenerator::setPointCloud(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > data)
{
  point_cloud_ = data;
};

}  //namespace
#endif /* TEMPLATE_GENERATOR_H_ */
