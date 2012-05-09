/*
 * trajectory_timing_generator.cpp
 *
 *  Created on: Dec 13, 2010
 *      Author: kalakris
 */

#include <robot_info/trajectory_timing_generator.h>
#include <usc_utilities/param_server.h>
#include <tf/transform_datatypes.h>

namespace robot_info
{

bool TrajectoryTimingGenerator::initialize()
{
  min_trajectory_duration_.resize(RobotInfo::NUM_ROBOT_PARTS);
  for (int i = 0; i < (int)RobotInfo::robot_part_names_.size(); ++i)
  {
    int robot_part_id = 0;
    ROS_VERIFY(RobotInfo::getRobotPartId(RobotInfo::robot_part_names_[i], robot_part_id));
    min_trajectory_duration_[robot_part_id] = 0.1;
  }
  node_handle_.reset(new ros::NodeHandle());
  readParams();
  return (initialized_ = true);
}

bool TrajectoryTimingGenerator::getMinTrajectoryDuration(const std::string& robot_part_name, double& min_trajectory_duration)
{
  int robot_part_id = 0;
  if(!RobotInfo::getRobotPartId(robot_part_name, robot_part_id))
  {
    return false;
  }
  min_trajectory_duration = min_trajectory_duration_[robot_part_id];
  return true;
}

double TrajectoryTimingGenerator::getMinCartesianTrajectoryDuration()
{
  return min_cartesian_trajectory_duration_;
}

double TrajectoryTimingGenerator::getDuration(const std::string& robot_part_name, const std::vector<double>& joint_angles1, const std::vector<double>& joint_angles2)
{
  checkInitialized();
  ROS_ASSERT(joint_angles1.size() == joint_angles2.size());
  double max_time = 0;
  if(!getMinTrajectoryDuration(robot_part_name, max_time))
  {
    ROS_ERROR("Could not compute trajectory timinings.");
    return false;
  }
  for (unsigned int i=0; i<joint_angles1.size(); ++i)
  {
    double dist = fabs(joint_angles1[i] - joint_angles2[i]);
    JointInfo joint_info;
    if(!RobotInfo::getJointInfo(robot_part_name, i, joint_info))
    {
      ROS_ERROR("Could not obtain joint info of robot part >%s< and index >%i<", robot_part_name.c_str(), i);
      return false;
    }
    double time = dist / joint_info.preferred_velocity_;
    if (time > max_time)
    {
      max_time = time;
    }
  }
  ROS_DEBUG("Movement time = %f", max_time);
  return max_time;
}

double TrajectoryTimingGenerator::getCartesianDuration(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
  checkInitialized();

  double max_time = min_cartesian_trajectory_duration_;

  double pos_dist[3];
  pos_dist[0] = pose1.position.x - pose2.position.x;
  pos_dist[1] = pose1.position.y - pose2.position.y;
  pos_dist[2] = pose1.position.z - pose2.position.z;

  for (int i=0; i<3; ++i)
  {
    pos_dist[i] = fabs(pos_dist[i]);
    double time = pos_dist[i] / cartesian_position_preferred_velocity_;
    if (time > max_time)
      max_time = time;
  }

  tf::Quaternion q1, q2;
  tf::quaternionMsgToTF(pose1.orientation, q1);
  tf::quaternionMsgToTF(pose2.orientation, q2);
  double angle = q1.angle(q2);
  double time = angle / cartesian_orientation_preferred_velocity_;
  if (time > max_time)
    max_time = time;

  ROS_DEBUG("Movement time = %f", max_time);
  return max_time;
}

void TrajectoryTimingGenerator::checkInitialized()
{
  if (!initialized_)
  {
    ROS_ERROR("TrajectoryTimingGenerator not initialized. Please call robot_info::init() before creating any objects from the robot_info library.");
    ROS_BREAK();
  }
}

void TrajectoryTimingGenerator::readParams()
{
  ROS_VERIFY(usc_utilities::read(*node_handle_, "/robot_model/trajectory_timing_generator/min_cartesian_trajectory_duration", min_cartesian_trajectory_duration_));
  ROS_VERIFY(usc_utilities::read(*node_handle_, "/robot_model/trajectory_timing_generator/cartesian_preferred_velocities/position", cartesian_position_preferred_velocity_));
  ROS_VERIFY(usc_utilities::read(*node_handle_, "/robot_model/trajectory_timing_generator/cartesian_preferred_velocities/orientation", cartesian_orientation_preferred_velocity_));
}

boost::shared_ptr<ros::NodeHandle> TrajectoryTimingGenerator::node_handle_;
bool TrajectoryTimingGenerator::initialized_=false;
std::vector<double> TrajectoryTimingGenerator::min_trajectory_duration_;

double TrajectoryTimingGenerator::min_cartesian_trajectory_duration_;
double TrajectoryTimingGenerator::cartesian_position_preferred_velocity_;
double TrajectoryTimingGenerator::cartesian_orientation_preferred_velocity_;

}
