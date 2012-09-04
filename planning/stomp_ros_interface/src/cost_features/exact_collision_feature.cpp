/*
 * exact_collision_feature.cpp
 *
 *  Created on: Jul 22, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cost_features/exact_collision_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_DECLARE_CLASS(stomp_ros_interface,
                        ExactCollisionFeature,
                        stomp_ros_interface::ExactCollisionFeature,
                        stomp_ros_interface::StompCostFeature);

namespace stomp_ros_interface
{

ExactCollisionFeature::ExactCollisionFeature():
    node_handle_("~")
{
  collision_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("exact_collision_markers", 128);
  collision_array_viz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("exact_collision_markers_array", 128);
  collision_color.a = 1.0;
  collision_color.r = 1.0;
  collision_color.g = 0.0;
  collision_color.b = 0.0;

  debug_collisions_ = false;
}

ExactCollisionFeature::~ExactCollisionFeature()
{
}

bool ExactCollisionFeature::initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group)
{
  return true;
}

int ExactCollisionFeature::getNumValues() const
{
  return 1;
}

void ExactCollisionFeature::getNames(std::vector<std::string>& names) const
{
  names.clear();
  names.push_back(getName());
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

  joint_angles_.resize(input->joint_angles_.rows());
  for (unsigned int i=0; i<input->joint_angles_.rows(); ++i)
    joint_angles_[i] = input->joint_angles_(i);

  input->per_rollout_data_->joint_state_group_->setKinematicState(joint_angles_);

  if (input->per_rollout_data_->collision_models_->isKinematicStateInCollision(*input->per_rollout_data_->kinematic_state_))
  {
    state_validity = false;
    feature_values[0] = 1.0;
    if (debug_collisions_)
    {
      visualization_msgs::MarkerArray arr;
      std::vector<arm_navigation_msgs::ContactInformation> contact_info;
      input->per_rollout_data_->collision_models_->getAllCollisionPointMarkers(*input->per_rollout_data_->kinematic_state_,
                                                                              arr, collision_color, ros::Duration(1.0));
      input->per_rollout_data_->collision_models_->getAllCollisionsForState(*input->per_rollout_data_->kinematic_state_,
                                                                           contact_info, 1);
      for (unsigned int i=0; i<contact_info.size(); ++i)
      {
        ROS_INFO("Collision between %s and %s", contact_info[i].contact_body_1.c_str(), contact_info[i].contact_body_2.c_str());
      }
      collision_array_viz_pub_.publish(arr);
    }
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
