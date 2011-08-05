/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

// ros includes
#include <angles/angles.h>

// #include <pluginlib/class_list_macros.h>

#include <pr2_controllers_msgs/JointControllerState.h>

// local incudes
#include <pr2_dynamic_movement_primitive_controller/joint_position_controller.h>

// PLUGINLIB_DECLARE_CLASS(dmp_motion_controllers, JointPositionController, dmp_controller::JointPositionController, pr2_controller_interface::Controller)

namespace pr2_dynamic_movement_primitive_controller
{

JointPositionController::JointPositionController() :
  initialized_(false), joint_state_(NULL), command_(0), commanded_effort_(0), publisher_rate_(1), publisher_counter_(0), publisher_buffer_size_(1000),
      error_(0.0), error_dot_(0.0), last_error_(0.0), robot_(NULL), last_time_(0)
{
}

JointPositionController::~JointPositionController()
{
}

bool JointPositionController::init(pr2_mechanism_model::RobotState *robot,
                                   ros::NodeHandle &node_handle)
{
  assert(robot);
  robot_ = robot;
  node_handle_ = node_handle;

  std::string joint_name;
  if (!node_handle_.getParam("joint", joint_name))
  {
    ROS_ERROR("Could not retrieve parameter >>joint<< in the namespace %s", node_handle_.getNamespace().c_str());
    initialized_ = false;
    return initialized_;
  }

  if (!pid_controller_.init(ros::NodeHandle(node_handle_, "pid")))
  {
    ROS_ERROR("Could not initialize pid controller.");
    initialized_ = false;
    return initialized_;
  }

  rosrt::init();
  pr2_controllers_msgs::JointControllerState joint_controller_state_msg;
  controller_state_publisher_.reset(new rosrt::Publisher<pr2_controllers_msgs::JointControllerState>(node_handle_.advertise<
      pr2_controllers_msgs::JointControllerState> (std::string("state"), 1), publisher_buffer_size_, joint_controller_state_msg));

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("JointPositionController could not find joint named %s.", joint_name.c_str());
    initialized_ = false;
    return initialized_;
  }
  if (!joint_state_->calibrated_)
  {
    ROS_ERROR("Joint %s not calibrated for JointPositionController.", joint_name.c_str());
    initialized_ = false;
    return initialized_;
  }

  initialized_ = true;
  return initialized_;
}

void JointPositionController::setGains(const double &p,
                                       const double &i,
                                       const double &d,
                                       const double &i_max,
                                       const double &i_min)
{
  pid_controller_.setGains(p, i, d, i_max, i_min);
}

void JointPositionController::getGains(double &p,
                                       double &i,
                                       double &d,
                                       double &i_max,
                                       double &i_min)
{
  pid_controller_.getGains(p, i, d, i_max, i_min);
}

std::string JointPositionController::getJointName()
{
  return joint_state_->joint_->name;
}

// Set the joint position command
void JointPositionController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current position command
void JointPositionController::getCommand(double & cmd)
{
  cmd = command_;
}

void JointPositionController::starting()
{
  pid_controller_.reset();
  command_ = joint_state_->position_;

  last_time_ = robot_->getTime();
  last_error_ = 0.0;
}

void JointPositionController::update()
{
  assert(joint_state_->joint_);

  error_ = 0.0;
  ros::Time time = robot_->getTime();
  dt_ = time - last_time_;

  if (joint_state_->joint_->type == urdf::Joint::REVOLUTE)
  {
    angles::shortest_angular_distance_with_limits(command_, joint_state_->position_, joint_state_->joint_->limits->lower, joint_state_->joint_->limits->upper,
                                                  error_);
  }
  else if (joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
  {
    error_ = angles::shortest_angular_distance(command_, joint_state_->position_);
  }
  else // prismatic
  {
    error_ = joint_state_->position_ - command_;
  }

  // calculate the derivative error
  if (dt_.toSec() != 0)
  {
    error_dot_ = (error_ - last_error_) / dt_.toSec();
    last_error_ = error_;
  }

  commanded_effort_ = pid_controller_.updatePid(error_, error_dot_, dt_);
  joint_state_->commanded_effort_ = commanded_effort_;

  publish();

  last_time_ = time;
}

/* DEBUG: Implement publish? */
void JointPositionController::publish()
{

  publisher_counter_++;
  if (publisher_counter_ % publisher_rate_ == 0)
  {
    pr2_controllers_msgs::JointControllerStatePtr joint_controller_state_msg;
    joint_controller_state_msg = controller_state_publisher_->allocate();
    if (joint_controller_state_msg)
    {
      joint_controller_state_msg->header.stamp = robot_->getTime();
      joint_controller_state_msg->set_point = command_;
      joint_controller_state_msg->process_value = joint_state_->position_;
      joint_controller_state_msg->process_value_dot = joint_state_->velocity_;
      joint_controller_state_msg->error = error_;
      joint_controller_state_msg->time_step = dt_.toSec();
      joint_controller_state_msg->command = commanded_effort_;

      double dummy;
      getGains(joint_controller_state_msg->p, joint_controller_state_msg->i, joint_controller_state_msg->d, joint_controller_state_msg->i_clamp, dummy);
      controller_state_publisher_->publish(joint_controller_state_msg);
    }
  }
}

}
