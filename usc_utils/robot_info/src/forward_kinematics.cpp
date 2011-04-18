/*
 * forward_kinematics.cpp
 *
 *  Created on: Dec 10, 2010
 *      Author: kalakris
 */

#include <robot_info/forward_kinematics.h>
#include <usc_utilities/assert.h>

namespace robot_info
{

bool ForwardKinematics::initialize(const std::string& start_link, const std::string& end_link)
{
  if(!chain_.initialize(start_link, end_link))
  {
    ROS_ERROR("Could not initialize kinematic chain from >%s< to >%s<.", start_link.c_str(), end_link.c_str());
    return false;
  }
  kdl_joint_array_.resize(chain_.getNumJoints());
  ROS_DEBUG("Initialized kinematic chain with >%d< joints", chain_.getNumJoints());
  return (initialized_ = true);
}

void ForwardKinematics::forwardKinematics(const std::vector<double>& joint_positions, geometry_msgs::Pose& pose)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(int(joint_positions.size()) == chain_.getNumJoints());
  KDL::Frame frame;
  for (int i=0; i<chain_.getNumJoints(); ++i)
  {
    kdl_joint_array_(i) = joint_positions[i];
  }
  chain_.forwardKinematics(kdl_joint_array_, frame);
  pose.position.x = frame.p.x();
  pose.position.y = frame.p.y();
  pose.position.z = frame.p.z();
  frame.M.GetQuaternion(pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w);
}

}
