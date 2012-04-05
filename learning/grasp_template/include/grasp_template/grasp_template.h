/*
 * grasp_template.h
 *
 *  Created on: Feb 14, 2011
 *      Author: herzog
 */

#ifndef GRASP_TEMPLATE_H_
#define GRASP_TEMPLATE_H_

#include <Eigen3/Eigen>

#include <visualization_msgs/Marker.h>

#include "template_heightmap.h"

namespace grasp_template
{

class GraspTemplate
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	/* create marker for visualization purposes
	 *
	 */
	visualization_msgs::Marker getVisualization() const;

	TemplateHeightmap heightmap_;
	Eigen3::Transform<double, 3, Eigen3::Affine> heightmap_frame_transform_;
};

}  //namespace
#endif /* GRASP_TEMPLATE_H_ */
