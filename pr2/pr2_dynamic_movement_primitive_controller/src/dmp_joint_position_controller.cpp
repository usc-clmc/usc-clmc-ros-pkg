/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks    ...

 \file   dmp_controller.h

 \author Alexander Herzog, Peter Pastor
 \date   Mar 1, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <robot_info/robot_info.h>
#include <usc_utilities/assert.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/dmp_joint_position_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_controller_implementation.h>

// import most common Eigen types
using namespace Eigen;

PLUGINLIB_DECLARE_CLASS(pr2_dynamic_movement_primitive_controller, DMPJointPositionController,
    pr2_dynamic_movement_primitive_controller::DMPJointPositionController, pr2_controller_interface::Controller)

namespace pr2_dynamic_movement_primitive_controller
{

bool DMPJointPositionController::init(pr2_mechanism_model::RobotState* robot_state,
                                      ros::NodeHandle& node_handle)
{
  ROS_INFO("Initializing DMP joint position controller.");

  // initialize joint position controllers
  joint_position_controllers_.clear();
  std::vector<std::string> joint_names;
  std::vector<std::string> controlled_joint_names;
  ROS_VERIFY(usc_utilities::read(node_handle, "joint_names", joint_names));
  for (int i = 0; i < (int)joint_names.size(); ++i)
  {
    controlled_joint_names.push_back(joint_names[i]);
    ros::NodeHandle joint_node_handle(node_handle, joint_names[i]);
    JointPositionController joint_controller;
    if (!joint_controller.init(robot_state, joint_node_handle))
    {
      ROS_ERROR("Could not initialize joint controller for joint >%s<.", joint_names[i].c_str());
      return false;
    };
    joint_position_controllers_.push_back(joint_controller);
  }

  std::string dmp_implementation;
  ROS_VERIFY(usc_utilities::read(node_handle, "dmp_implementation", dmp_implementation));
  if (dmp_implementation == "ICRA2009DMPControllerImplementation")
  {
    dmp_controller_.reset(new DMPControllerImplementation<dmp::ICRA2009DMP> ());
  }
  else
  {
    ROS_ERROR("Could not figure out witch DMPController implementation to chose.");
    return false;
  }

  ROS_VERIFY(dmp_controller_->initialize(node_handle.getNamespace(), controlled_joint_names));

  // initialize member
  num_joints_ = controlled_joint_names.size();
  desired_positions_ = Eigen::VectorXd::Zero(num_joints_);
  desired_velocities_ = Eigen::VectorXd::Zero(num_joints_);
  desired_accelerations_ = Eigen::VectorXd::Zero(num_joints_);

  ROS_DEBUG("Initialized >%s< with >%i< joints.", dmp_implementation.c_str(), num_joints_);
  for (int i = 0; i < num_joints_; ++i)
  {
    ROS_DEBUG(">%s<", controlled_joint_names[i].c_str());
  }

  return true;
}

// REAL-TIME REQUIREMENTS
void DMPJointPositionController::starting()
{
  ROS_DEBUG("Starting...");
  for (int i = 0; i < static_cast<int> (joint_position_controllers_.size()); ++i)
  {
    joint_position_controllers_[i].starting();
  }
  holdPositions();
}

// REAL-TIME REQUIREMENTS
void DMPJointPositionController::update()
{
  if (dmp_controller_->newDMPReady())
  {
    // set start of DMP to current desired position
    ROS_VERIFY(dmp_controller_->changeDMPStart(desired_positions_));
  }

  if (dmp_controller_->isRunning(desired_positions_, desired_velocities_, desired_accelerations_))
  {
    setDesiredState();
  }
  else
  {
    holdPositions();
  }

  for (int i = 0; i < static_cast<int> (joint_position_controllers_.size()); i++)
  {
    joint_position_controllers_[i].update();
  }
}

// REAL-TIME REQUIREMENTS
void DMPJointPositionController::setDesiredState()
{
  for (int i = 0; i < num_joints_; i++)
  {
    joint_position_controllers_[i].setCommand(desired_positions_(i));
  }
}

// REAL-TIME REQUIREMENTS
void DMPJointPositionController::holdPositions()
{
  getDesiredPosition();
  desired_velocities_.setZero(num_joints_);
  desired_accelerations_.setZero(num_joints_);
  setDesiredState();
}

// REAL-TIME REQUIREMENTS
void DMPJointPositionController::getDesiredPosition()
{
  for (int local_joint = 0; local_joint < num_joints_; ++local_joint)
  {
    desired_positions_(local_joint) = joint_position_controllers_[local_joint].getJointPosition();
  }
}

}
