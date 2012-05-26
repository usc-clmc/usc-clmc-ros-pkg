/*
 * stomp_optimization_task.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/stomp_optimization_task.h>
#include <usc_utilities/param_server.h>

namespace stomp_ros_interface
{

StompOptimizationTask::StompOptimizationTask(ros::NodeHandle node_handle):
    node_handle_(node_handle)
{
}

StompOptimizationTask::~StompOptimizationTask()
{
}

bool StompOptimizationTask::initialize(int num_threads)
{
  usc_utilities::read(node_handle_, "num_time_steps", num_time_steps_);
  usc_utilities::read(node_handle_, "movement_duration", movement_duration_);
  usc_utilities::read(node_handle_, "planning_group", planning_group_);
  usc_utilities::read(node_handle_, "reference_frame", reference_frame_);

  per_thread_data_.resize(num_threads);
  double max_radius_clearance = 0.0;
  for (int i=0; i<num_threads; ++i)
  {
    per_thread_data_[i].robot_model_.reset(new StompRobotModel());
    per_thread_data_[i].robot_model_->init(reference_frame_);
    per_thread_data_[i].planning_group_ = per_thread_data_[i].robot_model_->getPlanningGroup(planning_group_);
    num_dimensions_ = per_thread_data_[i].planning_group_->num_joints_;
    max_radius_clearance = per_thread_data_[i].robot_model_->getMaxRadiusClearance();
  }
  collision_space_.reset(new StompCollisionSpace());
  collision_space_->init(max_radius_clearance, reference_frame_);
  for (int i=0; i<num_threads; ++i)
  {
    per_thread_data_[i].cost_function_input_.resize(num_time_steps_);
    for (int t=0; t<num_time_steps_; ++t)
    {
      per_thread_data_[i].cost_function_input_[t].reset(new StompCostFunctionInput(
          collision_space_, per_thread_data_[i].robot_model_, per_thread_data_[i].planning_group_));
    }
  }

  // create the derivative costs
  std::vector<Eigen::MatrixXd> derivative_costs(num_dimensions_,
                                                Eigen::MatrixXd(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING, stomp::NUM_DIFF_RULES));
  std::vector<Eigen::VectorXd> initial_trajectory(num_dimensions_,
                                                  Eigen::VectorXd::Zero(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING));

  for (int d=0; d<num_dimensions_; ++d)
  {
    derivative_costs[d].col(stomp::STOMP_ACCELERATION) = Eigen::VectorXd::Ones(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    initial_trajectory[d] = Eigen::VectorXd::Zero(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
  }

  policy_.reset(new stomp::CovariantMovementPrimitive());
  policy_->initialize(num_time_steps_,
                      num_dimensions_,
                      movement_duration_,
                      derivative_costs,
                      initial_trajectory);

  return true;
}

bool StompOptimizationTask::execute(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id)
{
  computeFeatures(parameters, per_thread_data_[thread_id].features_, thread_id);
  return true;
}

void StompOptimizationTask::computeFeatures(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::MatrixXd& features,
                     int thread_id)
{
  // prepare the cost function input
  for (int t=0; t<num_time_steps_; ++t)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      per_thread_data_[thread_id].cost_function_input_[t]->joint_angles_(d) = parameters[d](t);
    }
    per_thread_data_[thread_id].cost_function_input_[t]->doFK(per_thread_data_[thread_id].planning_group_->fk_solver_);
  }

}

bool StompOptimizationTask::getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy)
{
  policy = policy_;
  return true;
}

bool StompOptimizationTask::setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy)
{
  policy_ = policy;
  return true;
}

double StompOptimizationTask::getControlCostWeight()
{
  return control_cost_weight_;
}

void StompOptimizationTask::setPlanningScene(const arm_navigation_msgs::PlanningScene& scene)
{
  collision_space_->setPlanningScene(scene);
}

} /* namespace stomp_ros_interface */
