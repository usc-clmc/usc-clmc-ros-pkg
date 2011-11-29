/*
 * robot_pose_visualizer.cpp
 *
 *  Created on: Jan 21, 2011
 *      Author: kalakris
 */

#include <arm_navigation_msgs/DisplayTrajectory.h>
#include <visualization_utilities/robot_pose_visualizer.h>
#include <robot_info/kdl_conversions.h>

using namespace robot_info;

namespace visualization_utilities
{

RobotPoseVisualizer::RobotPoseVisualizer(const std::string& topic_name)
{
  pub_ = node_handle_.advertise<arm_navigation_msgs::DisplayTrajectory>(topic_name, 100, true);
}

void RobotPoseVisualizer::publishPose(const std::string& robot_part_name, const std::string& frame_id, const std::vector<double> joint_angles)
{
  arm_navigation_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.model_id = "arm"; //TODO: check whether that can be anything?
  display_trajectory.trajectory.joint_trajectory.header.frame_id = frame_id;
  display_trajectory.trajectory.joint_trajectory.header.stamp = ros::Time::now();
  std::vector<std::string> joint_names;
  ROS_VERIFY(RobotInfo::getNames(robot_part_name, joint_names));
  display_trajectory.trajectory.joint_trajectory.joint_names = joint_names;
  display_trajectory.trajectory.joint_trajectory.points.resize(1);
  display_trajectory.trajectory.joint_trajectory.points[0].positions = joint_angles;
  display_trajectory.robot_state.joint_state = joint_state_monitor_.getJointStateRealJoints();
  pub_.publish(display_trajectory);
}

void RobotPoseVisualizer::publishPose(const std::string& robot_part_name, const std::string& frame_id, const KDL::JntArray& joint_angles)
{
  std::vector<double> joint_array;
  kdlToVector(joint_angles, joint_array);
  publishPose(robot_part_name, frame_id, joint_array);
}

}
