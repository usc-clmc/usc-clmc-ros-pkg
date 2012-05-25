/*
 * stomp_optimization_task.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/stomp_optimization_task.h>

namespace stomp_ros_interface
{

StompOptimizationTask::StompOptimizationTask()
{

}

StompOptimizationTask::~StompOptimizationTask()
{
}

bool StompOptimizationTask::initialize(int num_threads)
{
  policy_.reset(new stomp::CovariantMovementPrimitive());
  per_thread_data_.resize(num_threads);
  for (int i=0; i<num_threads; ++i)
  {
    per_thread_data_[i].robot_model_.reset(new StompRobotModel());
    per_thread_data_[i].robot_model_->init("BASE");
  }
  return true;
}

bool StompOptimizationTask::execute(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id)
{
  return true;
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

} /* namespace stomp_ros_interface */
