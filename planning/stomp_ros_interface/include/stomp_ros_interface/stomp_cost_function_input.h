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

class StompCostFunctionInput: public learnable_cost_function::Input
{
public:
  StompCostFunctionInput(boost::shared_ptr<StompCollisionSpace const> collision_space,
                         boost::shared_ptr<StompRobotModel const> robot_model,
                         const StompRobotModel::StompPlanningGroup* planning_group);
  virtual ~StompCostFunctionInput();

  virtual int getNumDimensions() const;
  virtual double getTime() const { return time_; }

  KDL::JntArray joint_angles_;
  std::vector<KDL::Vector> joint_axis_;
  std::vector<KDL::Vector> joint_pos_;
  std::vector<KDL::Frame> segment_frames_;
  std::vector<KDL::Vector> collision_point_pos_;
  std::vector<KDL::Vector> collision_point_vel_;
  std::vector<KDL::Vector> collision_point_acc_;
  double time_;

  boost::shared_ptr<StompCollisionSpace const> collision_space_;
  boost::shared_ptr<StompRobotModel const> robot_model_;
  const StompRobotModel::StompPlanningGroup* planning_group_;

  void doFK(boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver);

private:
  bool full_fk_done_;
};

} /* namespace stomp_ros_interface */
#endif /* STOMP_COST_FUNCTION_INPUT_H_ */
