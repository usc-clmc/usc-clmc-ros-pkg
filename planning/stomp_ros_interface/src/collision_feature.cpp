/*
 * collision_feature.cpp
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/collision_feature.h>

namespace stomp_ros_interface
{

CollisionFeature::CollisionFeature()
{
}

CollisionFeature::~CollisionFeature()
{
}

bool CollisionFeature::initialize(XmlRpc::XmlRpcValue& config)
{
  return true;
}

int CollisionFeature::getNumValues() const
{
  return 2; // 1 for smooth cost, 1 for state validity
}

void CollisionFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
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

  state_validity = true;

  // for each collision point, add up the distance field costs...
  double total_cost = 0.0;
  for (unsigned int i=0; i<input->collision_point_pos_.size(); ++i)
  {
    double potential = 0.0;
    bool valid = input->collision_space_->getCollisionPointPotential(
        input->planning_group_->collision_points_[i], input->collision_point_pos_[i], potential);
    total_cost += potential;
    if (!valid)
      state_validity = false;
  }
  feature_values[0] = total_cost;
  feature_values[1] = state_validity?0.0:1.0;

  // TODO gradients not computed yet!!!
}

std::string CollisionFeature::getName() const
{
  return "CollisionFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> CollisionFeature::clone() const
{
  boost::shared_ptr<CollisionFeature> ret(new CollisionFeature());
  return ret;
}

} /* namespace stomp_ros_interface */
