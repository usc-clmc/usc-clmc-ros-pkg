/*
 * trajectory_timing_generator.h
 *
 *  Created on: Dec 13, 2010
 *      Author: kalakris
 */

#ifndef TRAJECTORY_TIMING_GENERATOR_H_
#define TRAJECTORY_TIMING_GENERATOR_H_

#include <geometry_msgs/Pose.h>
#include <robot_info/robot_info.h>

namespace robot_info
{

/**
 * Generates a reasonable duration for a trajectory (cartesian / joint-space)
 */
class TrajectoryTimingGenerator
{
public:
  static bool initialize();

  static bool getMinTrajectoryDuration(const std::string& robot_part_name, double& min_trajectory_duration);
  static double getMinCartesianTrajectoryDuration();

  static double getDuration(const std::string& robot_part_name, const std::vector<double>& joint_angles1, const std::vector<double>& joint_angles2);
  static double getCartesianDuration(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

private:
  static boost::shared_ptr<ros::NodeHandle> node_handle_;
  static bool initialized_;

  static std::vector<double> min_trajectory_duration_;

  static double min_cartesian_trajectory_duration_;
  static double cartesian_position_preferred_velocity_;
  static double cartesian_orientation_preferred_velocity_;

  TrajectoryTimingGenerator(); // do not construct
  static void checkInitialized();
  static void readParams();
};

}

#endif /* TRAJECTORY_TIMING_GENERATOR_H_ */
