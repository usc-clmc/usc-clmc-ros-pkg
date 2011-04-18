/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_learner_utilities.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 27, 2011

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_LEARNER_UTILITIES_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_LEARNER_UTILITIES_H_

// system includes
#include <string>
#include <ros/ros.h>

#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/DMPUtilitiesMsg.h>
#include <dynamic_movement_primitive/TypeMsg.h>

// local includes
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_learner.h>

namespace dmp_utilities
{

class DynamicMovementPrimitiveLearnerUtilities
{

public:

  /*! Constructor
   */
  DynamicMovementPrimitiveLearnerUtilities() :
    initialized_(false) {};

  /*! Constructor
   */
  virtual ~DynamicMovementPrimitiveLearnerUtilities() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(ros::NodeHandle& node_handle);

  /*!
   * @param abs_bag_file_name
   * @param dmp_utilities_msg
   * @param robot_part_names
   * @param type
   * @return True on success, otherwise False
   */
  bool learnDMPsFromTrajectory(const std::string& abs_bag_file_name,
                               const dynamic_movement_primitive::TypeMsg& type,
                               const std::vector<std::string>& robot_part_names,
                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param type
   * @param robot_part_names_from_min_jerk
   * @param initial_duration
   * @param start
   * @param goal
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                            const std::vector<std::string>& robot_part_names_from_min_jerk,
                            const double initial_duration,
                            const Eigen::VectorXd& start,
                            const Eigen::VectorXd& goal,
                            dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                            const std::vector<std::string>& robot_part_names_from_min_jerk,
                            const double initial_duration,
                            const std::vector<double>& start,
                            const std::vector<double>& goal,
                            dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param type
   * @param robot_part_names_from_min_jerk
   * @param initial_durations
   * @param waypoints
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                            const std::vector<std::string>& robot_part_names_from_min_jerk,
                            const std::vector<double>& initial_durations,
                            const std::vector<Eigen::VectorXd>& waypoints,
                            dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                            const std::vector<std::string>& robot_part_names_from_min_jerk,
                            const std::vector<double>& initial_durations,
                            const std::vector<std::vector<double> >& waypoints,
                            dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param type
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param start
   * @param goal
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                         const dynamic_movement_primitive::TypeMsg& type,
                                         const std::vector<std::string>& robot_part_names_from_trajectory,
                                         const std::vector<std::string>& robot_part_names_from_min_jerk,
                                         const Eigen::VectorXd& start,
                                         const Eigen::VectorXd& goal,
                                         dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                         const dynamic_movement_primitive::TypeMsg& type,
                                         const std::vector<std::string>& robot_part_names_from_trajectory,
                                         const std::vector<std::string>& robot_part_names_from_min_jerk,
                                         const std::vector<double>& start,
                                         const std::vector<double>& goal,
                                         dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param type
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param duration_fractions
   * @param waypoints
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                         const dynamic_movement_primitive::TypeMsg& type,
                                         const std::vector<std::string>& robot_part_names_from_trajectory,
                                         const std::vector<std::string>& robot_part_names_from_min_jerk,
                                         const std::vector<double>& duration_fractions,
                                         const std::vector<Eigen::VectorXd>& waypoints,
                                         dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                         const dynamic_movement_primitive::TypeMsg& type,
                                         const std::vector<std::string>& robot_part_names_from_trajectory,
                                         const std::vector<std::string>& robot_part_names_from_min_jerk,
                                         const std::vector<double>& duration_fractions,
                                         const std::vector<std::vector<double> >& waypoints,
                                         dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                           const std::vector<std::string>& robot_part_names,
                           dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param start
   * @param goal
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                           const std::vector<std::string>& robot_part_names_from_trajectory,
                           const std::vector<std::string>& robot_part_names_from_min_jerk,
                           const Eigen::VectorXd& start,
                           const Eigen::VectorXd& goal,
                           dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                           const std::vector<std::string>& robot_part_names_from_trajectory,
                           const std::vector<std::string>& robot_part_names_from_min_jerk,
                           const std::vector<double>& start,
                           const std::vector<double>& goal,
                           dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param waypoints
   * @param duration_fractions
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                           const std::vector<std::string>& robot_part_names_from_trajectory,
                           const std::vector<std::string>& robot_part_names_from_min_jerk,
                           const std::vector<Eigen::VectorXd>& waypoints,
                           const std::vector<double>& duration_fractions,
                           dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                           const std::vector<std::string>& robot_part_names_from_trajectory,
                           const std::vector<std::string>& robot_part_names_from_min_jerk,
                           const std::vector<std::vector<double> >& waypoints,
                           const std::vector<double>& duration_fractions,
                           dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                               const std::vector<std::string>& robot_part_names,
                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param start
   * @param goal
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                               const std::vector<std::string>& robot_part_names_from_trajectory,
                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                               const Eigen::VectorXd& start,
                               const Eigen::VectorXd& goal,
                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                               const std::vector<std::string>& robot_part_names_from_trajectory,
                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                               const std::vector<double>& start,
                               const std::vector<double>& goal,
                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param waypoints
   * @param duration_fractions
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                               const std::vector<std::string>& robot_part_names_from_trajectory,
                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                               const std::vector<Eigen::VectorXd>& waypoints,
                               const std::vector<double>& duration_fractions,
                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                               const std::vector<std::string>& robot_part_names_from_trajectory,
                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                               const std::vector<std::vector<double> >& waypoints,
                               const std::vector<double>& duration_fractions,
                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names,
                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param start
   * @param goal
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                       const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                       const std::vector<double>& start,
                                       const std::vector<double>& goal,
                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

  /*!
   * @param abs_bag_file_name
   * @param robot_part_names_from_trajectory
   * @param robot_part_names_from_min_jerk
   * @param waypoints
   * @param duration_fractions
   * @param dmp_utilities_msg
   * @return True on success, otherwise False
   */
  bool learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                       const std::vector<Eigen::VectorXd>& waypoints,
                                       const std::vector<double>& duration_fractions,
                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);
  bool learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                       const std::vector<std::vector<double> >& waypoints,
                                       const std::vector<double>& duration_fractions,
                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg);

private:

  /*!
   */
  bool initialized_;

  /*!
   */
  ros::NodeHandle node_handle_;

};


}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_LEARNER_UTILITIES_H_ */
