/*
 * stomp_cost_function_input.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/stomp_cost_function_input.h>

namespace stomp_ros_interface
{

StompCostFunctionInput::StompCostFunctionInput(boost::shared_ptr<StompCollisionSpace const> collision_space,
                       boost::shared_ptr<StompRobotModel const> robot_model,
                       const StompRobotModel::StompPlanningGroup* planning_group)
{
  collision_space_ = collision_space;
  robot_model_ = robot_model;
  planning_group_ = planning_group;

  // allocate memory
  int nj = planning_group_->num_joints_;
  int nc = planning_group_->collision_points_.size();
  int nl = planning_group_->fk_solver_->getSegmentNames().size();
  joint_angles_ = KDL::JntArray(planning_group_->num_joints_);
  joint_axis_.resize(nj);
  joint_pos_.resize(nj);
  segment_frames_.resize(nl);
  collision_point_pos_.resize(nc);
  collision_point_vel_.resize(nc);
  collision_point_acc_.resize(nc);
  full_fk_done_ = false;
}

StompCostFunctionInput::~StompCostFunctionInput()
{
}

int StompCostFunctionInput::getNumDimensions() const
{
  return planning_group_->num_joints_;
}

void StompCostFunctionInput::doFK(boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver)
{
//  if (!full_fk_done_)
  {
    fk_solver->JntToCartFull(joint_angles_, joint_pos_, joint_axis_, segment_frames_);
    full_fk_done_ = true;
  }
//  else
//  {
//    fk_solver->JntToCartPartial(joint_angles_, joint_pos_, joint_axis_, segment_frames_);
//  }

  for (unsigned int i=0; i<planning_group_->collision_points_.size(); ++i)
  {
    planning_group_->collision_points_[i].getTransformedPosition(segment_frames_, collision_point_pos_[i]);
  }
}


} /* namespace stomp_ros_interface */
