/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks      ...

  \file         grasp_template_params.cpp

  \author       Alexander Herzog
  \date         April 1, 2012

 *********************************************************************/

#include <ros/ros.h>
#include <grasp_template/grasp_template_params.h>

namespace grasp_template
{

double GraspTemplateParams::template_width_ = 0.0;
bool GraspTemplateParams::initialized_ = false;

GraspTemplateParams::GraspTemplateParams()
{
  initialize();
  ros::param::get("~frame_view_point", frame_view_point_);
  ros::param::get("~frame_base", frame_base_);
  ros::param::get("~gripper_bounding_corner1_x", abb_corner_1_.x());
  ros::param::get("~gripper_bounding_corner1_y", abb_corner_1_.y());
  ros::param::get("~gripper_bounding_corner1_z", abb_corner_1_.z());
  ros::param::get("~gripper_bounding_corner2_x", abb_corner_2_.x());
  ros::param::get("~gripper_bounding_corner2_y", abb_corner_2_.y());
  ros::param::get("~gripper_bounding_corner2_z", abb_corner_2_.z());
  ros::param::get("~default_viewpoint_x", default_viewpoint_.x());
  ros::param::get("~default_viewpoint_y", default_viewpoint_.y());
  ros::param::get("~default_viewpoint_z", default_viewpoint_.z());
}

void GraspTemplateParams::actuatorBoundingBox(Eigen::Vector3d& e1, Eigen::Vector3d& e2) const
{
  e1 = abb_corner_1_;
  e2 = abb_corner_2_;
}


void GraspTemplateParams::initialize()
{
  ros::param::get("~template_width", template_width_);
  initialized_ = true;
}

double GraspTemplateParams::getTemplateWidth()
{
  if (!initialized_)
  {
    initialize();
  }

  return template_width_;
}

}  // namespace
