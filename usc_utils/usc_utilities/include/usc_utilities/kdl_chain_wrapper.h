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

#ifndef UTILITIES_KDL_CHAIN_WRAPPER_H_
#define UTILITIES_KDL_CHAIN_WRAPPER_H_

// system includes
#include <boost/shared_ptr.hpp>

// ros includes
#include <ros/ros.h>

#include <urdf/model.h>

#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <sensor_msgs/JointState.h>

// local includes

namespace usc_utilities
{

/**
 * Creates a KDL chain and provides forward kinematics functions on it
 *
 * Hides "mimic" joints from the user.
 *
 * \warning This class is not thread-safe!
 */
class KDLChainWrapper
{
public:
  KDLChainWrapper();
  virtual ~KDLChainWrapper();

  /**
   * Initialize the KDL Chain wrapper
   * @param node_handle
   * @param root_frame
   * @param tip_frame
   * @return
   */
  bool initialize(const std::string& root_frame, const std::string& tip_frame);

  /**
   * Gets the number of joints in the chain (excluding mimic joints)
   * @return
   */
  int getNumJoints();


  /**
   * Perform forward kinematics (w/ velocities) for an input JointState message
   * @param joint_state
   * @param frame_vel
   * @return
   */
  //bool forwardKinematicsVel(const sensor_msgs::JointState& joint_state, KDL::FrameVel& frame_vel);

  /**
   * Perform forward kinematics (w/ velocities) for an input JntArrayVel
   * @param jnt_array_vel
   * @param frame_vel
   * @return
   */
  //bool forwardKinematicsVel(const KDL::JntArrayVel& jnt_array_vel, KDL::FrameVel& frame_vel);

  /**
   * Perform forward kinematics for an input JointState message
   * @param joint_state
   * @param frame_vel
   * @return
   */
  //bool forwardKinematics(const sensor_msgs::JointState& joint_state, KDL::Frame& frame);

  /**
   * Perform forward kinematics for an input joint array
   * @param jnt_array
   * @param frame
   * @return
   */
  bool forwardKinematics(const KDL::JntArray& jnt_array, KDL::Frame& frame);
  bool forwardKinematics(const std::vector<double>& jnt_array, KDL::Frame& frame);

  /**
   * Converts a sensor_msgs::JointState ROS message into a KDL::JntArrayVel object
   * @param joint_state
   * @param jnt_array_vel
   * @return
   */
  //bool jointStateMsgToJntArrayVel(const sensor_msgs::JointState& joint_state, KDL::JntArrayVel& jnt_array_vel) const;

  /**
   * Converts a sensor_msgs::JointState ROS message into a KDL::JntArray object
   * @param joint_state
   * @param jnt_array_vel
   * @return
   */
  //bool jointStateMsgToJntArray(const sensor_msgs::JointState& joint_state, KDL::JntArray& jnt_array) const;

  void getJointNames(std::vector<std::string>& joint_names);

  void debugJointNames();

  void getChain(KDL::Chain& kdl_chain);

  void getTree(KDL::Tree& kdl_tree);

private:

  struct JointInfo
  {
    int mimic_joint;
    double multiplier;
    double offset;
  };

  void createJointNameMappings();
  bool initMimicJoints();

  bool initialized_;
  std::string tip_frame_;
  std::string root_frame_;
  ros::NodeHandle node_handle_;

  urdf::Model urdf_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::JntArray real_joint_array;

  boost::shared_ptr<KDL::ChainFkSolverVel> jnt_to_pose_vel_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;

  std::map<std::string, int> real_joint_name_to_index_;
  std::vector<std::string> real_joint_names_;
  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_name_to_index_;

  std::vector<JointInfo> mimic_joints_;

  int num_real_joints_;
  int num_joints_;      /**< num total joints excluding mimic joints */
  int num_mimic_joints_;

  void jointArrayToRealJointArray(const std::vector<double>& joint_array, KDL::JntArray& real_joint_array);
  void jointArrayToRealJointArray(const KDL::JntArray& joint_array, KDL::JntArray& real_joint_array);

  int getRealJointIndex(std::string& name);

};

}

#endif /* UTILITIES_KDL_CHAIN_WRAPPER_H_ */
