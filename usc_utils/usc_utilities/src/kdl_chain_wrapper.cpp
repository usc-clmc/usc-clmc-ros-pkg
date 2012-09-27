/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#include <usc_utilities/kdl_chain_wrapper.h>
#include <usc_utilities/assert.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/assert.h>
#include <map>

namespace usc_utilities
{

KDLChainWrapper::KDLChainWrapper()
{
}

KDLChainWrapper::~KDLChainWrapper()
{
}

int KDLChainWrapper::getNumJoints()
{
  ROS_ASSERT(initialized_);
  return num_joints_;
}

bool KDLChainWrapper::initialize(const std::string& root_frame, const std::string& tip_frame)
{
  root_frame_ = root_frame;
  tip_frame_ = tip_frame;

  // get URDF
  std::string robot_description;
  if(!node_handle_.getParam("/robot_description", robot_description))
  {
    ROS_ERROR("Could not retrieve >>robot_description<< from paramserver");
    return false;
  }

  if (!urdf_.initString(robot_description))
  {
    ROS_ERROR("Failed to parse urdf from /robot_description");
    return false;
  }

  // create kdl tree
  if (!kdl_parser::treeFromString(robot_description, kdl_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree.");
    return false;
  }

  // create the chain given the tree
  if (!kdl_tree_.getChain(root_frame_, tip_frame_, kdl_chain_))
  {
    ROS_ERROR("Failed to get kdl chain from %s to %s.", root_frame_.c_str(), tip_frame_.c_str());
    return false;
  }
  num_real_joints_ = kdl_chain_.getNrOfJoints();

  // create the joint to pose solver
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_pose_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

  // create urdf name to kdl joint index mapping and vice versa:
  createJointNameMappings();

  // initialize mimic joints
  if (!initMimicJoints())
  {
    ROS_ERROR("Could not initialize mimic joints.");
    return false;
  }

  real_joint_array.resize(num_real_joints_);

  return (initialized_ = true);
}

bool KDLChainWrapper::initMimicJoints()
{
  mimic_joints_.clear();
  num_mimic_joints_ = 0;
  joint_names_.clear();
  num_joints_ = 0;

  std::vector<int> real_joint_to_index_;

  for (int i=0; i<num_real_joints_; ++i)
  {
    JointInfo mimic_joint;
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_.getJoint(real_joint_names_[i]);
    if (!urdf_joint)
    {
      ROS_ERROR("Joint %s not found in urdf!", real_joint_names_[i].c_str());
      return false;
    }
    if (urdf_joint->mimic)
    {
      //ROS_INFO("joint %s mimics %s", joint_number_to_name_[i].c_str(), urdf_joint->mimic->joint_name.c_str());
      mimic_joint.multiplier = urdf_joint->mimic->multiplier;
      mimic_joint.offset = urdf_joint->mimic->offset;
      mimic_joint.mimic_joint = getRealJointIndex(urdf_joint->mimic->joint_name);
      if (mimic_joint.mimic_joint < 0)
      {
        //ROS_ERROR("Joint %s being mimicked by %s not found in this chain.", urdf_joint->mimic->joint_name.c_str(), real_joint_names_[i].c_str());
        return false;
      }
      ++num_mimic_joints_;
      real_joint_to_index_.push_back(-1);
    }
    else
    {
      mimic_joint.mimic_joint = i; // mimic itself by default
      mimic_joint.offset = 0.0;
      mimic_joint.multiplier = 1.0;
      joint_names_.push_back(real_joint_names_[i]);
      joint_name_to_index_.insert(std::make_pair(real_joint_names_[i], num_joints_));
      real_joint_to_index_.push_back(num_joints_);
      ++num_joints_;
    }
    mimic_joints_.push_back(mimic_joint);
  }

  // now fix the mimic joint indices
  for (int i=0; i<num_real_joints_; ++i)
  {
    mimic_joints_[i].mimic_joint = real_joint_to_index_[mimic_joints_[i].mimic_joint];
    //ROS_INFO("Real joint %d mimics joint %d", i, mimic_joints_[i].mimic_joint);
  }

  return true;
}

void KDLChainWrapper::createJointNameMappings()
{
  real_joint_names_.clear();
  real_joint_name_to_index_.clear();

  int joint_number=0;
  for (int i=0; i<int(kdl_chain_.segments.size()); ++i)
  {
    const KDL::Segment& segment = kdl_chain_.segments[i];
    const KDL::Joint& joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      std::string name = segment.getJoint().getName();
      real_joint_names_.push_back(name);
      real_joint_name_to_index_.insert(std::make_pair(name, joint_number));


      ++joint_number;
    }
  }
}

/*bool KDLChainWrapper::forwardKinematicsVel(const sensor_msgs::JointState& joint_state, KDL::FrameVel& frame_vel)
{
  ROS_ASSERT(initialized_);

  KDL::JntArrayVel jnt_array_vel(num_real_joints_);
  ROS_ASSERT_FUNC(jointStateMsgToJntArrayVel(joint_state, jnt_array_vel));
  jnt_to_pose_vel_solver_->JntToCart(jnt_array_vel, frame_vel);
  return true;
}

bool KDLChainWrapper::forwardKinematicsVel(const KDL::JntArrayVel& jnt_array_vel, KDL::FrameVel& frame_vel)
{
  ROS_ASSERT(initialized_);
  jnt_to_pose_vel_solver_->JntToCart(jnt_array_vel, frame_vel);
  return true;
}

bool KDLChainWrapper::forwardKinematics(const sensor_msgs::JointState& joint_state, KDL::Frame& frame)
{
  ROS_ASSERT(initialized_);

  KDL::JntArray jnt_array(num_real_joints_);
  ROS_ASSERT_FUNC(jointStateMsgToJntArray(joint_state, jnt_array));
  jnt_to_pose_solver_->JntToCart(jnt_array, frame);

  return true;
}*/

bool KDLChainWrapper::forwardKinematics(const KDL::JntArray& jnt_array, KDL::Frame& frame)
{
  ROS_ASSERT(initialized_);
  jointArrayToRealJointArray(jnt_array, real_joint_array);
  return (jnt_to_pose_solver_->JntToCart(real_joint_array, frame) >= 0);
}

bool KDLChainWrapper::forwardKinematics(const std::vector<double>& jnt_array, KDL::Frame& frame)
{
  ROS_ASSERT(initialized_);
  jointArrayToRealJointArray(jnt_array, real_joint_array);
  return (jnt_to_pose_solver_->JntToCart(real_joint_array, frame) >= 0);
}

/*bool KDLChainWrapper::jointStateMsgToJntArrayVel(const sensor_msgs::JointState& joint_state, KDL::JntArrayVel& jnt_array_vel) const
{
  ROS_ASSERT(initialized_);

  unsigned int joints_found = 0;

  // loop through the joint state message and fill up the jnt_array_vel
  for (int i=0; i<int(joint_state.name.size()); ++i)
  {
    std::map<std::string, int>::const_iterator it = real_joint_name_to_index_.find(joint_state.name[i]);
    if (it==real_joint_name_to_index_.end())
      continue;
    ++joints_found;
    int index = it->second;
    jnt_array_vel.q(index) = joint_state.position[i];
    jnt_array_vel.qdot(index) = joint_state.velocity[i];
  }
  if (joints_found!=jnt_array_vel.q.rows())
  {
    ROS_WARN("KDLChainWrapper::jointStateMsgToJntArrayVel - only %d out of %d joints were assigned", joints_found, jnt_array_vel.q.rows());
  }
  return true;
}*/

/*bool KDLChainWrapper::jointStateMsgToJntArray(const sensor_msgs::JointState& joint_state, KDL::JntArray& jnt_array) const
{
  ROS_ASSERT(initialized_);

  unsigned int joints_found = 0;

  // loop through the joint state message and fill up the jnt_array_vel
  for (int i=0; i<int(joint_state.name.size()); ++i)
  {
    std::map<std::string, int>::const_iterator it = real_joint_name_to_index_.find(joint_state.name[i]);
    if (it==real_joint_name_to_index_.end())
      continue;
    ++joints_found;
    int index = it->second;
    jnt_array(index) = joint_state.position[i];
  }
  if (joints_found!=jnt_array.rows())
  {
    ROS_WARN("KDLChainWrapper::jointStateMsgToJntArray - only %d out of %d joints were assigned", joints_found, jnt_array.rows());
  }
  return true;
}*/

void KDLChainWrapper::getJointNames(std::vector<std::string>& joint_names)
{
  joint_names = joint_names_;
}

void KDLChainWrapper::debugJointNames()
{
  for (int i=0; i<num_joints_; ++i)
  {
    ROS_INFO("Joint %d: %s", i, joint_names_[i].c_str());
  }
}

int KDLChainWrapper::getRealJointIndex(std::string& name)
{
  std::map<std::string, int>::iterator it = real_joint_name_to_index_.find(name);
  if (it==real_joint_name_to_index_.end())
    return -1;
  return it->second;
}

void KDLChainWrapper::jointArrayToRealJointArray(const std::vector<double>& joint_array, KDL::JntArray& real_joint_array)
{
  ROS_ASSERT(int(joint_array.size()) == num_joints_);
  ROS_ASSERT(int(real_joint_array.rows()) == num_real_joints_);
  for (int i=0; i<num_real_joints_; ++i)
  {
    real_joint_array(i) = mimic_joints_[i].offset + mimic_joints_[i].multiplier *
        joint_array[mimic_joints_[i].mimic_joint];
  }
}

void KDLChainWrapper::jointArrayToRealJointArray(const KDL::JntArray& joint_array, KDL::JntArray& real_joint_array)
{
  ROS_ASSERT(int(joint_array.rows()) == num_joints_);
  ROS_ASSERT(int(real_joint_array.rows()) == num_real_joints_);
  for (int i=0; i<num_real_joints_; ++i)
  {
    real_joint_array(i) = mimic_joints_[i].offset + mimic_joints_[i].multiplier *
        joint_array(mimic_joints_[i].mimic_joint);
  }
}

void KDLChainWrapper::getChain(KDL::Chain& kdl_chain)
{
  kdl_chain = kdl_chain_;
}

void KDLChainWrapper::getTree(KDL::Tree& kdl_tree)
{
  kdl_tree = kdl_tree_;
}

}
