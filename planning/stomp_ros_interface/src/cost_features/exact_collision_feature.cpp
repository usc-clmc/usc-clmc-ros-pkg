/*
 * exact_collision_feature.cpp
 *
 *  Created on: Jul 22, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cost_features/exact_collision_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

namespace stomp_ros_interface
{

ExactCollisionFeature::ExactCollisionFeature()
{
}

ExactCollisionFeature::~ExactCollisionFeature()
{
}

bool ExactCollisionFeature::initialize(XmlRpc::XmlRpcValue& config)
{
  return true;
}

int ExactCollisionFeature::getNumValues() const
{
  return 1;
}

void ExactCollisionFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
                               bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity)
{
  boost::shared_ptr<stomp_ros_interface::StompCostFunctionInput const> input =
      boost::dynamic_pointer_cast<stomp_ros_interface::StompCostFunctionInput const>(generic_input);

  // initialize arrays
  feature_values.clear();
  feature_values.resize(getNumValues(), 0.0);
  if (compute_gradients)
  {
    gradients.resize(getNumValues(), Eigen::VectorXd::Zero(input->getNumDimensions()));
  }

  if (input->per_thread_data_->collision_models_->isKinematicStateInCollision(*input->per_thread_data_->kinematic_state_))
  {
    state_validity = false;
    feature_values[0] = 1.0;
  }
  else
  {
    state_validity = true;
  }

}

std::string ExactCollisionFeature::getName() const
{
  return "ExactCollisionFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> ExactCollisionFeature::clone() const
{
  boost::shared_ptr<ExactCollisionFeature> ret(new ExactCollisionFeature());
  return ret;
}

} /* namespace stomp_ros_interface */
