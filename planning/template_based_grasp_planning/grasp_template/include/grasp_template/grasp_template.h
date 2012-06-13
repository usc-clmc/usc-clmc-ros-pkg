/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_template.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef GRASP_TEMPLATE_H_
#define GRASP_TEMPLATE_H_

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <grasp_template/Heightmap.h>
#include <grasp_template/template_heightmap.h>

namespace grasp_template
{

class GraspTemplate
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspTemplate(){};
  GraspTemplate(const Heightmap& hm, const geometry_msgs::Pose& pose);

  TemplateHeightmap heightmap_;
  Eigen::Vector3d object_to_template_translation_;
  Eigen::Quaterniond object_to_template_rotation_;

  void getPose(geometry_msgs::Pose& pose) const;

  /*
   * create marker for visualization purposes
   */
  std::vector<visualization_msgs::Marker> getVisualization(const std::string& ns,
      const std::string& frame_id, int id = 0) const;
};

} //namespace
#endif /* GRASP_TEMPLATE_H_ */
