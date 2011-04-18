/*
 * robot_pose_visualizer.h
 *
 *  Created on: Jan 21, 2011
 *      Author: kalakris
 */

#ifndef ROBOT_POSE_VISUALIZER_H_
#define ROBOT_POSE_VISUALIZER_H_

#include <ros/ros.h>
#include <planning_environment/monitors/joint_state_monitor.h>
#include <kdl/jntarray.hpp>

#include <robot_info/robot_info.h>

namespace visualization_utilities
{

class RobotPoseVisualizer
{
public:
  RobotPoseVisualizer(const std::string& topic_name);
  virtual ~RobotPoseVisualizer() {};

  void publishPose(const std::string& robot_part_name, const std::string& frame_id, const std::vector<double> joint_angles);
  void publishPose(const std::string& robot_part_name, const std::string& frame_id, const KDL::JntArray& joint_angles);

private:
  ros::NodeHandle node_handle_;
  ros::Publisher pub_;
  planning_environment::JointStateMonitor joint_state_monitor_;

};

}

#endif /* ROBOT_POSE_VISUALIZER_H_ */
