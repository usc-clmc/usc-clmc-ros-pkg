/*
 * stomp_cost_function_input.h
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#ifndef STOMP_COST_FUNCTION_INPUT_H_
#define STOMP_COST_FUNCTION_INPUT_H_

#include <learnable_cost_function/input.h>
#include <kdl/frames.hpp>
#include <stomp_ros_interface/stomp_collision_space.h>
#include <stomp_ros_interface/stomp_robot_model.h>

namespace stomp_ros_interface
{

class StompCostFunctionInput: learnable_cost_function::Input
{
public:
  StompCostFunctionInput();
  virtual ~StompCostFunctionInput();

protected:
  KDL::JntArray joint_angles_;
  std::vector<KDL::Vector> joint_axis_;
  std::vector<KDL::Vector> joint_pos_;
  std::vector<KDL::Frame> segment_frames_;
  std::vector<KDL::Vector> collision_point_pos_;
  std::vector<KDL::Vector> collision_point_vel_;
  std::vector<KDL::Vector> collision_point_acc_;

  boost::shared_ptr<StompCollisionSpace const> collision_space_;
  boost::shared_ptr<StompRobotModel const> robot_model_;
  boost::shared_ptr<StompRobotModel::StompPlanningGroup const> planning_group_;
};

} /* namespace stomp_ros_interface */
#endif /* STOMP_COST_FUNCTION_INPUT_H_ */
