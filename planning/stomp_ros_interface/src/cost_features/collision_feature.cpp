/*
 * collision_feature.cpp
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cost_features/collision_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>
#include <stomp_ros_interface/sigmoid.h>
#include <sstream>

PLUGINLIB_DECLARE_CLASS(stomp_ros_interface,
                        CollisionFeature,
                        stomp_ros_interface::CollisionFeature,
                        stomp_ros_interface::StompCostFeature);

namespace stomp_ros_interface
{

CollisionFeature::CollisionFeature()
{
  sigmoid_centers_.push_back(-0.025);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.0);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.025);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.05);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.075);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.1);
  sigmoid_slopes_.push_back(200.0);

  num_sigmoids_ = sigmoid_centers_.size();
  //num_sigmoids_ = 0;
}

CollisionFeature::~CollisionFeature()
{
}

bool CollisionFeature::initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group)
{
  return true;
}

int CollisionFeature::getNumValues() const
{
  return 1 + num_sigmoids_; // 1 for smooth cost, rest for sigmoids
}

void CollisionFeature::getNames(std::vector<std::string>& names) const
{
  names.clear();
  names.push_back(getName()+"/DistanceFieldCost");
  for (int i=0; i<num_sigmoids_; ++i)
  {
    std::stringstream ss;
    ss << getName() << "/Sigmoid_" << sigmoid_centers_[i];
    names.push_back(ss.str());
  }
}

void CollisionFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
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

  state_validity = true;

  // for each collision point, add up the distance field costs...
  double total_cost = 0.0;
  for (unsigned int i=0; i<input->collision_point_pos_.size(); ++i)
  {

//    double potential = 0.0;
//    bool in_collision = input->collision_space_->getCollisionPointPotential(
//        input->planning_group_->collision_points_[i], input->collision_point_pos_[i], potential);

    double distance = 0.0;
    bool in_collision = input->collision_space_->getCollisionPointDistance(
        input->planning_group_->collision_points_[i], input->collision_point_pos_[i], distance);

    double potential = 0.0;
    double clearance = input->planning_group_->collision_points_[i].getClearance();
    if (distance >= clearance)
      potential = 0.0;
    else if (distance >= 0.0)
      potential = 0.5 * (distance - clearance) * (distance - clearance) / clearance;
    else // distance < 0.0
      potential = -distance + 0.5 * clearance;

    double vel_mag = input->collision_point_vel_[i].Norm();
    total_cost += potential * vel_mag;
    if (in_collision)
      state_validity = false;

    for (int i=0; i<num_sigmoids_; ++i)
    {
      double val = (1.0 - sigmoid(distance, sigmoid_centers_[i], sigmoid_slopes_[i]));
      feature_values[i+1] += val * vel_mag;
      //printf("distance = %f, sigmoid %d = %f\n", distance, i, val);
    }

  }
  feature_values[0] = total_cost;
  //feature_values[1] = state_validity?0.0:1.0;

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
