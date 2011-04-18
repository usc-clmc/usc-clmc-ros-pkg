/*
 * robot_monitor.cpp
 *
 *  Created on: Nov 30, 2010
 *      Author: mrinal
 */

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <robot_info/robot_monitor.h>
#include <robot_info/robot_info.h>

namespace robot_info
{

RobotMonitor::RobotMonitor(int downsample_factor):
    joint_state_received_(false),
    downsample_factor_(downsample_factor),
    downsample_counter_(downsample_factor),
    paused_(false),
    user_callback_enabled_(false)
{
  joint_state_sub_ = node_handle_.subscribe("/joint_states", downsample_factor,
                                             &RobotMonitor::jointStateCallback, this);
  parsed_joint_state_.resize(RobotInfo::getNumJoints());

  const std::vector<std::string> robot_part_names = RobotInfo::getRobotPartNames();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    std::vector<int> joint_ids;
    if(!RobotInfo::getJointIds(robot_part_names[i], joint_ids))
    {
      ROS_ERROR("Could not obtain joint ids. Cannot create RobotMonitor.");
      ROS_BREAK();
    }
  }

}

void RobotMonitor::registerCallback(boost::function<void (const sensor_msgs::JointState::ConstPtr& msg)> callback)
{
  user_callback_ = callback;
  user_callback_enabled_ = true;
}

void RobotMonitor::pause()
{
  paused_ = true;
}

void RobotMonitor::resume()
{
  paused_ = false;
}

bool RobotMonitor::getJointState(sensor_msgs::JointState& joint_state)
{
  boost::mutex::scoped_lock lock(joint_state_mutex_);
  joint_state = joint_state_;
  return joint_state_received_;
}

void RobotMonitor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (!paused_ && (++downsample_counter_ >= downsample_factor_))
  {
    downsample_counter_ = 0;
    joint_state_mutex_.lock();
    joint_state_ = *msg;
    joint_state_received_ = true;
    joint_state_parsed_ = false;
    joint_state_mutex_.unlock();
    if (user_callback_enabled_)
    {
      user_callback_(msg);
    }
  }
}

bool RobotMonitor::parseJointState()
{
  boost::mutex::scoped_lock lock(joint_state_mutex_);
  if (!joint_state_received_)
  {
    return false;
  }
  if (!joint_state_parsed_)
  {
    for (int i=0; i<int(joint_state_.name.size()); ++i)
    {
      int sl_id = RobotInfo::getJointId(joint_state_.name[i]);
      if (sl_id == -1)
      {
        continue;
      }
      parsed_joint_state_[sl_id].position = joint_state_.position[i];
      parsed_joint_state_[sl_id].velocity = joint_state_.velocity[i];
      parsed_joint_state_[sl_id].effort = joint_state_.effort[i];
    }
    joint_state_parsed_ = true;
  }
  return true;
}

bool RobotMonitor::getJointPositions(const std::string& robot_part_name, std::vector<double>& joint_array)
{
  std::vector<int> joint_ids;
  if(!RobotInfo::getJointIds(robot_part_name, joint_ids))
  {
    return false;
  }
  return getJointPositions(joint_ids, joint_array);
}

bool RobotMonitor::getJointPositions(const std::vector<int>& ids, std::vector<double>& positions)
{
  if (!parseJointState())
  {
    return false;
  }
  positions.resize(ids.size());
  for (unsigned int i=0; i<ids.size(); ++i)
  {
    positions[i] = parsed_joint_state_[ids[i]].position;
  }
  return true;
}

bool RobotMonitor::getJointStates(const std::vector<int>& ids, std::vector<JointState>& states)
{
  if (!parseJointState())
  {
    return false;
  }
  states.resize(ids.size());
  for (unsigned int i=0; i<ids.size(); ++i)
  {
    states[i] = parsed_joint_state_[ids[i]];
  }
  return true;
}

}
