/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks      ...

  \file         grasp_template_params.h

  \author       Alexander Herzog
  \date         April 1, 2012

 *********************************************************************/

#ifndef GRASP_TEMPLATE_PARAMS_H_
#define GRASP_TEMPLATE_PARAMS_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace grasp_template
{

class GraspTemplateParams
{
public:

  GraspTemplateParams();

  const std::string& frameViewPoint() const { return frame_view_point_;};
  const std::string& frameBase() const {return frame_base_;};
  void actuatorBoundingBox(Eigen::Vector3d& e1, Eigen::Vector3d& e2) const;
  const Eigen::Vector3d& getDefaultViewpoint() const { return default_viewpoint_;};
  static double getTemplateWidth();

private:

  static double template_width_;
  static bool initialized_;

  std::string frame_view_point_, frame_base_;
  Eigen::Vector3d abb_corner_1_;
  Eigen::Vector3d abb_corner_2_;
  Eigen::Vector3d default_viewpoint_;

  static void initialize();
};

}  //namespace
#endif /* GRASP_TEMPLATE_PARAMS_H_ */
