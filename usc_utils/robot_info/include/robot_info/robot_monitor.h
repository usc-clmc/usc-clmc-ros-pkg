/*
 * robot_monitor.h
 *
 *  Created on: Nov 30, 2010
 *      Author: mrinal
 */

#ifndef ROBOT_MONITOR_H_
#define ROBOT_MONITOR_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>
#include <robot_info/robot_info.h>
#include <boost/function.hpp>
#include <robot_info/robot_info.h>

namespace robot_info
{

/**
 * A simple struct that holds position, velocity and effort (torque) of a joint
 */
struct JointState
{
  double position;
  double velocity;
  double effort;
};

/**
 * Class that listens to sensor_msgs::JointState messages.
 */
class RobotMonitor
{
public:
  /**
   * Creates a RobotMonitor
   * @param downsample_factor Update rate is down-sampled by this positive integer (defaults to 1)
   */
  RobotMonitor(int downsample_factor=1);
  virtual ~RobotMonitor() {};

  /**
   * Register a callback function which will be called every time joint states are updated
   * @param f
   */
  void registerCallback(boost::function<void (const sensor_msgs::JointState::ConstPtr& msg)> callback);

  /**
   * Stops updating joint states and calling callbacks
   */
  void pause();

  /**
   * Resumes updating joint states and calling callbacks
   */
  void resume();

  /**
   * Get the last updated JointState message
   * @param joint_state JointState message
   * @return false if no JointState has been received yet
   */
  bool getJointState(sensor_msgs::JointState& joint_state);

  bool getJointPositions(const std::string& robot_part_name, std::vector<double>& joint_array);
  bool getHeadJointPositions(std::vector<double>& joint_array);
  bool getRightArmJointPositions(std::vector<double>& joint_array);
  bool getRightHandJointPositions(std::vector<double>& joint_array);

  bool getHeadJointStates(std::vector<JointState>& joint_state_array);
  bool getRightArmJointStates(std::vector<JointState>& joint_state_array);
  bool getRightHandJointStates(std::vector<JointState>& joint_state_array);

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joint_state_sub_;

  bool joint_state_received_;
  sensor_msgs::JointState joint_state_;
  std::vector<JointState> parsed_joint_state_; /**< 0-based joint indexing */
  bool joint_state_parsed_;
  boost::mutex joint_state_mutex_;

  int downsample_factor_;
  int downsample_counter_;

  bool paused_;

  bool user_callback_enabled_;
  boost::function<void (const sensor_msgs::JointState::ConstPtr& msg)> user_callback_;

  std::vector<std::vector<int> > joint_ids_;

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  bool parseJointState(); /**< Parses joint state msg into parsed_joint_state_. Returns false if joint states not received yet */

  bool getJointPositions(const std::vector<int>& ids, std::vector<double>& positions);
  bool getJointStates(const std::vector<int>& ids, std::vector<JointState>& states);

};

}

#endif /* ROBOT_MONITOR_H_ */
