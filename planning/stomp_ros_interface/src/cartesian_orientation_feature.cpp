/*
 * cartesian_orientation_feature.cpp
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cartesian_orientation_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

namespace stomp_ros_interface
{

CartesianOrientationFeature::CartesianOrientationFeature()
{
}

CartesianOrientationFeature::~CartesianOrientationFeature()
{
}

bool CartesianOrientationFeature::initialize(XmlRpc::XmlRpcValue& config)
{
  return true;
}

int CartesianOrientationFeature::getNumValues() const
{
  return 1;
}

void CartesianOrientationFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
                               bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity)
{
  boost::shared_ptr<stomp_ros_interface::StompCostFunctionInput const> input =
      boost::dynamic_pointer_cast<stomp_ros_interface::StompCostFunctionInput const>(generic_input);

  // initialize arrays
  feature_values.resize(getNumValues(), 0.0);
  if (compute_gradients)
  {
    gradients.resize(getNumValues(), Eigen::VectorXd::Zero(input->getNumDimensions()));
  }

  // abs dot product of end effector orient and cart velocity
  int i = input->collision_point_pos_.size()-1;
  int j = input->planning_group_->end_effector_segment_index_;
  KDL::Vector orient_vector = input->segment_frames_[j].M.UnitZ();
  KDL::Vector velocity_vector = input->collision_point_vel_[i];
  velocity_vector.Normalize();
  feature_values[0] = fabs(KDL::dot(orient_vector, velocity_vector));

}

std::string CartesianOrientationFeature::getName() const
{
  return "CartesianOrientationFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> CartesianOrientationFeature::clone() const
{
  boost::shared_ptr<CartesianOrientationFeature> ret(new CartesianOrientationFeature());
  return ret;
}

} /* namespace stomp_ros_interface */
