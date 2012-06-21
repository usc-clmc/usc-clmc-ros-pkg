/*
 * chomp.cpp
 *
 *  Created on: Jun 20, 2012
 *      Author: kalakris
 */

#include <stomp/chomp.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

namespace stomp
{

CHOMP::CHOMP()
{

}

CHOMP::~CHOMP()
{
}

bool CHOMP::initialize(ros::NodeHandle& node_handle, boost::shared_ptr<Task> task)
{
  node_handle_ = node_handle;
  ROS_VERIFY(readParameters());

  task_ = task;
  task_->getPolicy(policy_);
  policy_->getNumTimeSteps(num_time_steps_);
  control_cost_weight_ = task_->getControlCostWeight();
  policy_->getNumDimensions(num_dimensions_);
  policy_->getControlCosts(control_costs_);
  policy_->getInvControlCosts(inv_control_costs_);
  policy_->getParameters(parameters_);
  update_.resize(num_dimensions_, Eigen::VectorXd(num_time_steps_));
  return true;
}

bool CHOMP::runSingleIteration(int iteration_number)
{
  policy_->getParameters(parameters_);
  task_->execute(parameters_, costs_, weighted_feature_values, iteration_number, -1,
                 0, true, gradients_);
  policy_->computeControlCostGradient(parameters_, control_cost_weight_, control_cost_gradients_);

  noiseless_rollout.parameters_ = parameters_;
  noiseless_rollout.state_costs_ = costs_;
  noiseless_rollout.total_cost_ = costs_.sum();

  for (int d=0; d<num_dimensions_; ++d)
  {
    update_[d] = -learning_rate_ * inv_control_costs_[d] * (gradients_[d] + control_cost_gradients_[d]);
    parameters_[d] += update_[d];
  }
  policy_->setParameters(parameters_);

  return true;
}

void CHOMP::getNoiselessRollout(Rollout& rollout)
{
  rollout = noiseless_rollout;
}

bool CHOMP::readParameters()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, std::string("learning_rate"), learning_rate_));
  return true;
}

} /* namespace stomp */
