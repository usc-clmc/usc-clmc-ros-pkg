/*
 * cartesian_vel_acc_feature.cpp
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cost_features/cartesian_vel_acc_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

namespace stomp_ros_interface
{

CartesianVelAccFeature::CartesianVelAccFeature()
{
}

CartesianVelAccFeature::~CartesianVelAccFeature()
{
}

bool CartesianVelAccFeature::initialize(XmlRpc::XmlRpcValue& config)
{
  return true;
}

int CartesianVelAccFeature::getNumValues() const
{
  return 2; // vel and acc for each joint
}

void CartesianVelAccFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
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

  int i = input->collision_point_pos_.size()-1;

  feature_values[0] = input->collision_point_vel_[i].Norm();
  feature_values[1] = input->collision_point_acc_[i].Norm();

  feature_values[0]*=feature_values[0];
  feature_values[1]*=feature_values[1];

}

std::string CartesianVelAccFeature::getName() const
{
  return "CartesianVelAccFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> CartesianVelAccFeature::clone() const
{
  boost::shared_ptr<CartesianVelAccFeature> ret(new CartesianVelAccFeature());
  return ret;
}

} /* namespace stomp_ros_interface */
