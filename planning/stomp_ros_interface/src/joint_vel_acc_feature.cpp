/*
 * joint_vel_acc_feature.cpp
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/joint_vel_acc_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

namespace stomp_ros_interface
{

JointVelAccFeature::JointVelAccFeature(int num_joints):
    num_joints_(num_joints)
{
}

JointVelAccFeature::~JointVelAccFeature()
{
}

bool JointVelAccFeature::initialize(XmlRpc::XmlRpcValue& config)
{
  return true;
}

int JointVelAccFeature::getNumValues() const
{
  return num_joints_*2; // vel and acc for each joint
}

void JointVelAccFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
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

  for (int j=0; j<num_joints_; ++j)
  {
    feature_values[j*2 + 0] = input->joint_angles_vel_(j)*input->joint_angles_vel_(j);
    feature_values[j*2 + 1] = input->joint_angles_acc_(j)*input->joint_angles_acc_(j);
  }

}

std::string JointVelAccFeature::getName() const
{
  return "JointVelAccFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> JointVelAccFeature::clone() const
{
  boost::shared_ptr<JointVelAccFeature> ret(new JointVelAccFeature(num_joints_));
  return ret;
}

} /* namespace stomp_ros_interface */
