/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		trajectory_utilities.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 5, 2011

 *********************************************************************/

#ifndef TRAJECTORY_UTILITIES_H_
#define TRAJECTORY_UTILITIES_H_

// system includes
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <Eigen/Eigen>

#include <filters/transfer_function.h>
#include <geometry_msgs/PoseStamped.h>

#include <dmp_lib/dynamic_movement_primitive.h>
#include <dmp_lib/trajectory.h>

// local includes

namespace dmp_utilities
{

/*! Abbreviation vor convinience
 */
typedef geometry_msgs::PoseStamped PoseStampedMsg;

class TrajectoryUtilities
{

public:

  /*!
   * @param dmp DMP must be setup.
   * @param base_frame_id
   * @param poses
   * @param rest_posture
   * @return True if success, otherwise False
   */
  static bool createTrajectory(const dmp_lib::DMPPtr dmp,
                               const std::string& base_frame_id,
                               std::vector<PoseStampedMsg>& poses,
                               std::vector<Eigen::VectorXd>& rest_postures,
                               const double movement_duration = -1.0);

  /*!
   * @param trajectory
   * @param variable_names
   * @param abs_bag_file_name
   * @param sampling_frequency
   * @param topic_name
   * @param compute_derivatives
   * @return True if success, otherwise False
   */
  static bool createJointStateTrajectory(dmp_lib::Trajectory& trajectory,
                                         const std::vector<std::string>& variable_names,
                                         const std::string& abs_bag_file_name,
                                         const double sampling_frequency,
                                         const std::string& topic_name = "/joint_states",
                                         const bool compute_derivatives = true);

  /*!
   * @param trajectory
   * @param wrench_variable_names
   * @param abs_bag_file_name
   * @param sampling_frequency
   * @param topic_name
   * @param compute_derivatives
   * @return True if success, otherwise False
   */
  static bool createWrenchTrajectory(dmp_lib::Trajectory& trajectory,
                                     const std::vector<std::string>& wrench_variable_names,
                                     const std::string& abs_bag_file_name,
                                     const double sampling_frequency,
                                     const std::string& topic_name,
                                     const bool compute_derivatives = true);

  /*!
   * @param pose_trajectory
   * @param joint_trajectory
   * @param start_link_name
   * @param end_link_name
   * @param variable_names
   * @return True if success, otherwise False
   */
  static bool createPoseTrajectory(dmp_lib::Trajectory& pose_trajectory,
                                   const dmp_lib::Trajectory& joint_trajectory,
                                   const std::string& start_link_name,
                                   const std::string& end_link_name,
                                   const std::vector<std::string>& variable_names);

  /*!
   * @param trajectory
   * @param time_stampes
   * @param compute_derivatives
   * @return True if success, otherwise False
   */
  static bool resample(dmp_lib::Trajectory& trajectory,
                       const std::vector<ros::Time>& time_stampes,
                       const double sampling_frequency,
                       const bool compute_derivatives = true);

  /*!
   * @param trajectory
   * @param filter_name
   * @return True if success, otherwise False
   */
  static bool filter(dmp_lib::Trajectory& trajectory,
                     const std::string& filter_name);

private:

  /*! Constructor
   */
  TrajectoryUtilities() {};

  /*! Destructor
   */
  virtual ~TrajectoryUtilities() {};

};

}

#endif /* TRAJECTORY_UTILITIES_H_ */
