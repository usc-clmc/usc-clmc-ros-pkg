/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_learner.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 5, 2011

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_LEARNER_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_LEARNER_H_

// system includes
#include <string>
#include <vector>

#include <ros/ros.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/constants.h>
#include <usc_utilities/param_server.h>

#include <robot_info/robot_info.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>
#include <dynamic_movement_primitive/TypeMsg.h>

// local includes
#include <dynamic_movement_primitive_utilities/trajectory_utilities.h>

namespace dmp_utilities
{

template<class DMPType>
  class DynamicMovementPrimitiveLearner
  {

  public:

    /*!
     * @param dmp
     * @param node_handle
     * @param abs_bag_file_name
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
  static bool learnJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                 ros::NodeHandle& node_handle,
                                 const std::string& abs_bag_file_name,
                                 const std::vector<std::string>& robot_part_names,
                                 const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    /*!
     * @param dmp
     * @param node_handle
     * @param type
     * @param robot_part_names
     * @param initial_duration
     * @param start
     * @param goal
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
  static bool learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                  ros::NodeHandle& node_handle,
                                  const dynamic_movement_primitive::TypeMsg& type,
                                  const std::vector<std::string>& robot_part_names,
                                  const double initial_duration,
                                  const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal,
                                  const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);
  static bool learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                  ros::NodeHandle& node_handle,
                                  const dynamic_movement_primitive::TypeMsg& type,
                                  const std::vector<std::string>& robot_part_names,
                                  const double initial_duration,
                                  const std::vector<double>& start,
                                  const std::vector<double>& goal,
                                  const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    /*!
     * @param dmp
     * @param node_handle
     * @param type
     * @param robot_part_names
     * @param initial_durations
     * @param waypoints
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                    ros::NodeHandle& node_handle,
                                    const dynamic_movement_primitive::TypeMsg& type,
                                    const std::vector<std::string>& robot_part_names,
                                    const std::vector<double>& initial_durations,
                                    const std::vector<Eigen::VectorXd>& waypoints,
                                    const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);
    static bool learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                    ros::NodeHandle& node_handle,
                                    const dynamic_movement_primitive::TypeMsg& type,
                                    const std::vector<std::string>& robot_part_names,
                                    const std::vector<double>& initial_durations,
                                    const std::vector<std::vector<double> >& waypoints,
                                    const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);


    /*!
     * @param dmp
     * @param node_handle
     * @param abs_bag_file_name
     * @param robot_part_names_from_trajectory
     * @param robot_part_names_from_min_jerk
     * @param start
     * @param goal
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool learnJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                   ros::NodeHandle& node_handle,
                                   const std::string& abs_bag_file_name,
                                   const std::vector<std::string>& robot_part_names_from_trajectory,
                                   const std::vector<std::string>& robot_part_names_from_min_jerk,
                                   const Eigen::VectorXd& start,
                                   const Eigen::VectorXd& goal,
                                   const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    static bool learnJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                   ros::NodeHandle& node_handle,
                                   const std::string& abs_bag_file_name,
                                   const std::vector<std::string>& robot_part_names_from_trajectory,
                                   const std::vector<std::string>& robot_part_names_from_min_jerk,
                                   const std::vector<Eigen::VectorXd>& waypoints,
                                   const std::vector<double>& duration_fractions,
                                   const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    /*!
     * @param dmp
     * @param node_handle
     * @param abs_bag_file_name
     * @param robot_part_names
     * @param start_link_name
     * @param end_link_name
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool learnCartesianSpaceDMP(typename DMPType::DMPPtr& dmp,
                                       ros::NodeHandle& node_handle,
                                       const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names,
                                       const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    /*!
     * @param dmp
     * @param node_handle
     * @param abs_bag_file_name
     * @param robot_part_names_from_trajectory
     * @param robot_part_names_from_min_jerk
     * @param start
     * @param goal
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool learnCartesianSpaceDMP(typename DMPType::DMPPtr& dmp,
                                       ros::NodeHandle& node_handle,
                                       const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                       const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    static bool learnCartesianSpaceDMP(typename DMPType::DMPPtr& dmp,
                                       ros::NodeHandle& node_handle,
                                       const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                       const std::vector<Eigen::VectorXd>& waypoints,
                                       const std::vector<double>& duration_fractions,
                                       const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    /*!
     * @param dmp
     * @param node_handle
     * @param abs_bag_file_name
     * @param robot_part_names
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool learnCartesianAndJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                               ros::NodeHandle& node_handle,
                                               const std::string& abs_bag_file_name,
                                               const std::vector<std::string>& robot_part_names,
                                               const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    /*!
     * @param dmp
     * @param node_handle
     * @param abs_bag_file_name
     * @param robot_part_names_from_trajectory
     * @param robot_part_names_from_min_jerk
     * @param start
     * @param goal
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool learnCartesianAndJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                               ros::NodeHandle& node_handle,
                                               const std::string& abs_bag_file_name,
                                               const std::vector<std::string>& robot_part_names_from_trajectory,
                                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                                               const Eigen::VectorXd& start,
                                               const Eigen::VectorXd& goal,
                                               const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    static bool learnCartesianAndJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                               ros::NodeHandle& node_handle,
                                               const std::string& abs_bag_file_name,
                                               const std::vector<std::string>& robot_part_names_from_trajectory,
                                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                                               const std::vector<Eigen::VectorXd>& waypoints,
                                               const std::vector<double>& duration_fractions,
                                               const double sampling_frequency = robot_info::RobotInfo::DEFAULT_SAMPLING_FREQUENCY);

    /*!
     * @param dmp
     * @param trajectory
     * @param abs_bag_file_name
     * @param robot_part_names
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool createJointStateTrajectory(typename DMPType::DMPPtr& dmp,
                                           dmp_lib::Trajectory& trajectory,
                                           const std::string& abs_bag_file_name,
                                           const std::vector<std::string>& robot_part_names,
                                           const double sampling_frequency);

    /*!
     * @param dmp
     * @param trajectory
     * @param abs_bag_file_name
     * @param robot_part_names
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool createWrenchTrajectory(typename DMPType::DMPPtr& dmp,
                                       dmp_lib::Trajectory& trajectory,
                                       const std::string& abs_bag_file_name,
                                       const std::vector<std::string>& robot_part_names,
                                       const double sampling_frequency);

    /*!
     * @param dmp
     * @param trajectory
     * @param abs_bag_file_name
     * @param robot_part_names
     * @param sampling_frequency
     * @return True if successful, otherwise False
     */
    static bool createAccelerationTrajectory(typename DMPType::DMPPtr& dmp,
                                             dmp_lib::Trajectory& trajectory,
                                             const std::string& abs_bag_file_name,
                                             const std::vector<std::string>& robot_part_names,
                                             const double sampling_frequency);

    /*!
     * @param dmp
     * @param duration_fractions
     * @param durations
     * @return True on success, otherwise false
     */
    static bool getDurations(typename DMPType::DMPPtr& dmp,
                             const std::vector<double>& duration_fractions,
                             std::vector<double>& durations)
    {
      double initial_duration = 0;
      ROS_VERIFY(dmp->getInitialDuration(initial_duration));
      return TrajectoryUtilities::getDurations(initial_duration, duration_fractions, durations);
    }

  private:

    /*! Constructor
     */
    DynamicMovementPrimitiveLearner() {};

    /*! Destructor
     */
    virtual ~DynamicMovementPrimitiveLearner() {};

  };

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::createJointStateTrajectory(typename DMPType::DMPPtr& dmp,
                                                                            dmp_lib::Trajectory& trajectory,
                                                                            const std::string& abs_bag_file_name,
                                                                            const std::vector<std::string>& robot_part_names,
                                                                            const double sampling_frequency)
  {
    if (robot_info::RobotInfo::containsJointParts(robot_part_names))
    {
      dmp_lib::Trajectory joint_trajectory;
      std::vector<std::string> dmp_joint_variable_names = dmp->getVariableNames();
      robot_info::RobotInfo::extractJointNames(dmp_joint_variable_names);
      ROS_VERIFY(TrajectoryUtilities::createJointStateTrajectory(joint_trajectory, dmp_joint_variable_names, abs_bag_file_name, sampling_frequency));

      if (trajectory.isInitialized())
      {
        ROS_VERIFY(trajectory.cutAndCombine(joint_trajectory));
      }
      else
      {
        trajectory = joint_trajectory;
      }
    }
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::createWrenchTrajectory(typename DMPType::DMPPtr& dmp,
                                                                        dmp_lib::Trajectory& trajectory,
                                                                        const std::string& abs_bag_file_name,
                                                                        const std::vector<std::string>& robot_part_names,
                                                                        const double sampling_frequency)
  {
    std::vector<std::string> parts = robot_part_names;
    robot_info::RobotInfo::extractWrenchParts(parts);
    if (robot_info::RobotInfo::containsWrenchParts(parts))
    {
      for (std::vector<std::string>::const_iterator vi = parts.begin(); vi != parts.end(); ++vi)
      {
        dmp_lib::Trajectory wrench_trajectory;
        std::vector<std::string> dmp_wrench_variable_names = dmp->getVariableNames();
        ROS_ERROR_COND(dmp_wrench_variable_names.empty(), "DMP does not contain any variable names. This should never happen.");
        ROS_DEBUG_COND(!dmp_wrench_variable_names.empty(), "Contained DMP variable names are:");
        for (int i = 0; i < (int)dmp_wrench_variable_names.size(); ++i)
        {
          ROS_DEBUG_COND(!dmp_wrench_variable_names.empty(), ">%s<", dmp_wrench_variable_names[i].c_str());
        }
        robot_info::RobotInfo::extractWrenchNames(dmp_wrench_variable_names);
        ROS_ERROR_COND(dmp_wrench_variable_names.empty(), "DMP does not contain any wrench variable names. This should never happen.");
        ROS_DEBUG_COND(!dmp_wrench_variable_names.empty(), "Contained wrench variable names are:");
        for (int i = 0; i < (int)dmp_wrench_variable_names.size(); ++i)
        {
          ROS_DEBUG_COND(!dmp_wrench_variable_names.empty(), ">%s<", dmp_wrench_variable_names[i].c_str());
        }
        std::string topic_name = "/SL/" + robot_info::RobotInfo::getWhichArmLowerLetterFromRobotPart(*vi) + "_hand_local_wrench_processed";
        ROS_VERIFY(TrajectoryUtilities::createWrenchTrajectory(wrench_trajectory, dmp_wrench_variable_names, abs_bag_file_name, sampling_frequency, topic_name));
        if (trajectory.isInitialized())
        {
          ROS_VERIFY(trajectory.cutAndCombine(wrench_trajectory));
        }
        else
        {
          trajectory = wrench_trajectory;
        }
      }
    }
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::createAccelerationTrajectory(typename DMPType::DMPPtr& dmp,
                                                                              dmp_lib::Trajectory& trajectory,
                                                                              const std::string& abs_bag_file_name,
                                                                              const std::vector<std::string>& robot_part_names,
                                                                              const double sampling_frequency)
  {
    std::vector<std::string> parts = robot_part_names;
    robot_info::RobotInfo::extractAccelerationParts(parts);
    if (robot_info::RobotInfo::containsAccelerationParts(parts))
    {
      for (std::vector<std::string>::const_iterator vi = parts.begin(); vi != parts.end(); ++vi)
      {
        dmp_lib::Trajectory acceleration_trajectory;
        std::vector<std::string> dmp_acceleration_variable_names = dmp->getVariableNames();

        ROS_ERROR_COND(dmp_acceleration_variable_names.empty(), "DMP does not contain any variable names. This should never happen.");
        ROS_DEBUG_COND(!dmp_acceleration_variable_names.empty(), "Contained DMP variable names are:");
        for (int i = 0; i < (int)dmp_acceleration_variable_names.size(); ++i)
        {
          ROS_DEBUG_COND(!dmp_acceleration_variable_names.empty(), ">%s<", dmp_acceleration_variable_names[i].c_str());
        }
        robot_info::RobotInfo::extractAccelerationNames(dmp_acceleration_variable_names);
        ROS_ERROR_COND(dmp_acceleration_variable_names.empty(), "DMP does not contain any acceleration variable names. This should never happen.");
        ROS_DEBUG_COND(!dmp_acceleration_variable_names.empty(), "Contained acceleration variable names are:");
        for (int i = 0; i < (int)dmp_acceleration_variable_names.size(); ++i)
        {
          ROS_DEBUG_COND(!dmp_acceleration_variable_names.empty(), ">%s<", dmp_acceleration_variable_names[i].c_str());
        }
        std::string topic_name = "/SL/" + robot_info::RobotInfo::getWhichArmLowerLetterFromRobotPart(*vi) + "_hand_accelerations_processed";
        ROS_VERIFY(TrajectoryUtilities::createAccelerationTrajectory(acceleration_trajectory, dmp_acceleration_variable_names, abs_bag_file_name, sampling_frequency, topic_name));
        if (trajectory.isInitialized())
        {
          ROS_VERIFY(trajectory.cutAndCombine(acceleration_trajectory));
        }
        else
        {
          trajectory = acceleration_trajectory;
        }
      }
    }
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                    ros::NodeHandle& node_handle,
                                                                    const std::string& abs_bag_file_name,
                                                                    const std::vector<std::string>& robot_part_names,
                                                                    const double sampling_frequency)
  {
    ROS_INFO("Learning joint space DMP from file >%s< with the following robot parts:", abs_bag_file_name.c_str());
    for (int i = 0; i < (int)robot_part_names.size(); ++i)
    {
      ROS_INFO("- >%s<", robot_part_names[i].c_str());
    }
    ros::NodeHandle joint_space_node_handle(node_handle, "joint_space_dmp");

    // initialize dmp from node handle
    ROS_VERIFY(DMPType::initFromNodeHandle(dmp, robot_part_names, joint_space_node_handle));

    // read joint space trajectory from bag file
    dmp_lib::Trajectory trajectory;

    ROS_VERIFY(DynamicMovementPrimitiveLearner<DMPType>::createJointStateTrajectory(dmp, trajectory, abs_bag_file_name, robot_part_names, sampling_frequency));
    ROS_VERIFY(DynamicMovementPrimitiveLearner<DMPType>::createWrenchTrajectory(dmp, trajectory, abs_bag_file_name, robot_part_names, sampling_frequency));
    ROS_VERIFY(DynamicMovementPrimitiveLearner<DMPType>::createAccelerationTrajectory(dmp, trajectory, abs_bag_file_name, robot_part_names, sampling_frequency));

    // learn dmp
    ROS_VERIFY(dmp->learnFromTrajectory(trajectory));
    dmp->changeType(dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE);
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                                                     ros::NodeHandle& node_handle,
                                                                     const dynamic_movement_primitive::TypeMsg& type,
                                                                     const std::vector<std::string>& robot_part_names,
                                                                     const double initial_duration,
                                                                     const Eigen::VectorXd& start,
                                                                     const Eigen::VectorXd& goal,
                                                                     const double sampling_frequency)
  {
    std::vector<double> initial_durations;
    initial_durations.push_back(initial_duration);
    std::vector<Eigen::VectorXd> waypoints;
    waypoints.push_back(start);
    waypoints.push_back(goal);
    return learnDMPFromMinJerk(dmp, node_handle, type, robot_part_names, initial_durations, waypoints, sampling_frequency);
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                                                     ros::NodeHandle& node_handle,
                                                                     const dynamic_movement_primitive::TypeMsg& type,
                                                                     const std::vector<std::string>& robot_part_names,
                                                                     const double initial_duration,
                                                                     const std::vector<double>& start,
                                                                     const std::vector<double>& goal,
                                                                     const double sampling_frequency)
  {
    return learnDMPFromMinJerk(dmp, node_handle, type, robot_part_names, initial_duration, Eigen::VectorXd::Map(&start[0], start.size()),
                               Eigen::VectorXd::Map(&goal[0], goal.size()), sampling_frequency);
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                                                     ros::NodeHandle& node_handle,
                                                                     const dynamic_movement_primitive::TypeMsg& type,
                                                                     const std::vector<std::string>& robot_part_names,
                                                                     const std::vector<double>& initial_durations,
                                                                     const std::vector<Eigen::VectorXd>& waypoints,
                                                                     const double sampling_frequency)
  {
    if(type.type == type.DISCRETE_JOINT_SPACE)
    {
      ROS_DEBUG("Learning joint space DMP from minimum jerk.");
      ros::NodeHandle joint_space_node_handle(node_handle, "joint_space_dmp");
      // initialize dmp from node handle
      ROS_VERIFY(DMPType::initFromNodeHandle(dmp, robot_part_names, joint_space_node_handle));
    }
    else if(type.type == type.DISCRETE_CARTESIAN_SPACE)
    {
      ROS_DEBUG("Learning cartesian space DMP from minimum jerk.");
      ros::NodeHandle cartesian_space_node_handle(node_handle, "cartesian_space_dmp");
      // initialize dmp from node handle
      ROS_VERIFY(DMPType::initFromNodeHandle(dmp, robot_part_names, cartesian_space_node_handle));
    }
    else
    {
      ROS_ERROR("Type >%i< is invalid for learning from minimum jerk.", type.type);
      return false;
    }

    // learn dmp
    ROS_VERIFY(dmp->learnFromMinimumJerk(waypoints, sampling_frequency, initial_durations));
    dmp->changeType(type.type);
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnDMPFromMinJerk(typename DMPType::DMPPtr& dmp,
                                                                     ros::NodeHandle& node_handle,
                                                                     const dynamic_movement_primitive::TypeMsg& type,
                                                                     const std::vector<std::string>& robot_part_names,
                                                                     const std::vector<double>& initial_durations,
                                                                     const std::vector<std::vector<double> >& waypoints,
                                                                     const double sampling_frequency)
  {
    std::vector<Eigen::VectorXd> eigen_waypoints;
    for (int i = 0; i < (int)waypoints.size(); ++i)
    {
      Eigen::VectorXd waypoint = Eigen::VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
      eigen_waypoints.push_back(waypoint);
    }
    return learnDMPFromMinJerk(dmp, node_handle, type, robot_part_names, initial_durations, eigen_waypoints, sampling_frequency);
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                    ros::NodeHandle& node_handle,
                                                                    const std::string& abs_bag_file_name,
                                                                    const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                    const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                    const Eigen::VectorXd& start,
                                                                    const Eigen::VectorXd& goal,
                                                                    const double sampling_frequency)
  {
    // learn joint space dmp
    typename DMPType::DMPPtr joint_dmp;
    ROS_VERIFY(learnJointSpaceDMP(joint_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // learn minimum jerk dmp
    double initial_duration = 0;
    ROS_VERIFY(joint_dmp->getInitialDuration(initial_duration));
    typename DMPType::DMPPtr min_jerk_dmp;
    dynamic_movement_primitive::TypeMsg type;
    type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE;
    ROS_VERIFY(learnDMPFromMinJerk(min_jerk_dmp, node_handle, type, robot_part_names_from_min_jerk, initial_duration, start, goal, sampling_frequency));

    // add the two dmps
    dmp = joint_dmp;
    ROS_VERIFY(dmp->add(*min_jerk_dmp, false));
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                    ros::NodeHandle& node_handle,
                                                                    const std::string& abs_bag_file_name,
                                                                    const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                    const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                    const std::vector<Eigen::VectorXd>& waypoints,
                                                                    const std::vector<double>& duration_fractions,
                                                                    const double sampling_frequency)
  {
    // learn joint space dmp
    typename DMPType::DMPPtr joint_dmp;
    ROS_VERIFY(learnJointSpaceDMP(joint_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // learn minimum jerk dmp
    double initial_duration = 0;
    ROS_VERIFY(joint_dmp->getInitialDuration(initial_duration));

    std::vector<double> durations;
    ROS_VERIFY(TrajectoryUtilities::getDurations(initial_duration, duration_fractions, durations));

    typename DMPType::DMPPtr min_jerk_dmp;
    dynamic_movement_primitive::TypeMsg type;
    type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE;
    ROS_VERIFY(learnDMPFromMinJerk(min_jerk_dmp, node_handle, type, robot_part_names_from_min_jerk, durations, waypoints, sampling_frequency));

    // add the two dmps
    dmp = joint_dmp;
    ROS_VERIFY(dmp->add(*min_jerk_dmp, false));
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnCartesianSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                        ros::NodeHandle& node_handle,
                                                                        const std::string& abs_bag_file_name,
                                                                        const std::vector<std::string>& robot_part_names,
                                                                        const double sampling_frequency)
  {
    std::string base_link_name;
    ROS_VERIFY(usc_utilities::read(node_handle, "base_link_name", base_link_name));
    ros::NodeHandle cartesian_space_node_handle(node_handle, "cartesian_space_dmp");
    ROS_INFO("Learning cartesian space DMP from file >%s< with the following robot parts:", abs_bag_file_name.c_str());
    bool first = true;
    for (int i = 0; i < (int)robot_part_names.size(); ++i)
    {
      ros::NodeHandle robot_part_node_handle(cartesian_space_node_handle, robot_part_names[i]);

      typename DMPType::DMPPtr tmp_dmp;
      std::string end_link_name;
      std::string pose_frame_id;

      bool from_joint_states = robot_part_node_handle.getParam("end_link_name", end_link_name);
      ROS_DEBUG_COND(from_joint_states, "Creating pose trajectory from joint state trajectory.");
      bool from_poses = robot_part_node_handle.getParam("pose_frame_id", pose_frame_id);
      ROS_DEBUG_COND(from_poses, "Reading pose trajectory from >%s<.", abs_bag_file_name.c_str());
      ROS_ASSERT_MSG((!from_joint_states && !from_poses) || !(from_joint_states && from_poses),
                     "Cannot learn trajectory from joint states AND pose trajectory.");

      std::vector<std::string> robot_part_names_for_cartesian_space;
      dmp_lib::Trajectory pose_trajectory;
      std::vector<double> offset(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT, 0.0);

      if (from_joint_states || from_poses)
      {
        ROS_INFO("Learning cartesian space DMP for robot part >%s<.", robot_part_names[i].c_str());
        robot_part_names_for_cartesian_space.push_back(robot_part_names[i]);
        // initialize dmp from node handle
        ROS_VERIFY(DMPType::initFromNodeHandle(tmp_dmp, robot_part_names_for_cartesian_space, cartesian_space_node_handle));
      }

      if (from_joint_states)
      {
        // read joint space trajectory from bag file
        std::vector<std::string> joint_variable_names;
        ROS_VERIFY(robot_info::RobotInfo::getArmJointNames(robot_part_names_for_cartesian_space, joint_variable_names));
        ROS_ASSERT_MSG(!joint_variable_names.empty(), "Joint variable names is empty. This should never happen.");
        for (int j = 0; j < (int)joint_variable_names.size(); ++j)
        {
          ROS_DEBUG("Joint variable name: >%s<.", joint_variable_names[j].c_str());
        }
        dmp_lib::Trajectory joint_trajectory;
        ROS_VERIFY(TrajectoryUtilities::createJointStateTrajectory(joint_trajectory, joint_variable_names, abs_bag_file_name, sampling_frequency));
        ROS_VERIFY(TrajectoryUtilities::createPoseTrajectory(pose_trajectory, offset, joint_trajectory, base_link_name, end_link_name, tmp_dmp->getVariableNames()));
      }

      if (from_poses)
      {
        std::vector<double> offset(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT, 0.0);
        std::string topic_name;
        ROS_VERIFY(usc_utilities::read(cartesian_space_node_handle, robot_part_names[i] + "/topic_name", topic_name));
        ROS_VERIFY(TrajectoryUtilities::readPoseTrajectory(pose_trajectory, offset, abs_bag_file_name, sampling_frequency, topic_name, tmp_dmp->getVariableNames()));
      }

      if (from_joint_states || from_poses)
      {
        ROS_VERIFY(pose_trajectory.computeDerivatives());
        // create debug trajectory for debugging purposes
        dmp_lib::TrajectoryPtr debug_trajectory(new dmp_lib::Trajectory());
        // learn dmp
        ROS_VERIFY(tmp_dmp->learnFromTrajectory(pose_trajectory, debug_trajectory));

        tmp_dmp->changeType(dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE);
        ROS_VERIFY(tmp_dmp->setInitialStart(offset));

        if(first)
        {
          first = false;
          dmp = tmp_dmp;
        }
        else
        {
          dmp->add(*tmp_dmp);
        }
      }
      else
      {
        ROS_DEBUG("Skipping learning cartesian space DMP for robot part >%s<.", robot_part_names[i].c_str());
      }
    }
    if(!dmp.get())
    {
      ROS_ERROR("Robot parts are missing. Could not obtain >end_link_name< from param server.");
      return false;
    }

    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnCartesianSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                        ros::NodeHandle& node_handle,
                                                                        const std::string& abs_bag_file_name,
                                                                        const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                        const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                        const Eigen::VectorXd& start,
                                                                        const Eigen::VectorXd& goal,
                                                                        const double sampling_frequency)
  {
    // learn cartesian space dmp
    typename DMPType::DMPPtr cartesian_dmp;
    ROS_VERIFY(learnCartesianSpaceDMP(cartesian_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // learn minimum jerk dmp
    double initial_duration = 0;
    ROS_VERIFY(cartesian_dmp->getInitialDuration(initial_duration));
    typename DMPType::DMPPtr min_jerk_dmp;
    dynamic_movement_primitive::TypeMsg type;
    type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE;
    ROS_VERIFY(learnDMPFromMinJerk(min_jerk_dmp, node_handle, type, robot_part_names_from_min_jerk, initial_duration, start, goal, sampling_frequency));

    // add the two dmps
    dmp = cartesian_dmp;
    ROS_VERIFY(dmp->add(*min_jerk_dmp, false));
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnCartesianSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                        ros::NodeHandle& node_handle,
                                                                        const std::string& abs_bag_file_name,
                                                                        const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                        const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                        const std::vector<Eigen::VectorXd>& waypoints,
                                                                        const std::vector<double>& duration_fractions,
                                                                        const double sampling_frequency)
  {
    // learn cartesian space dmp
    typename DMPType::DMPPtr cartesian_dmp;
    ROS_VERIFY(learnCartesianSpaceDMP(cartesian_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // learn minimum jerk dmp
    double initial_duration = 0;
    ROS_VERIFY(cartesian_dmp->getInitialDuration(initial_duration));

    std::vector<double> durations;
    ROS_VERIFY(TrajectoryUtilities::getDurations(initial_duration, duration_fractions, durations));

    typename DMPType::DMPPtr min_jerk_dmp;
    dynamic_movement_primitive::TypeMsg type;
    type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE;
    ROS_VERIFY(learnDMPFromMinJerk(min_jerk_dmp, node_handle, type, robot_part_names_from_min_jerk, durations, waypoints, sampling_frequency));

    // add the two dmps
    dmp = cartesian_dmp;
    ROS_VERIFY(dmp->add(*min_jerk_dmp, false));
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnCartesianAndJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                                ros::NodeHandle& node_handle,
                                                                                const std::string& abs_bag_file_name,
                                                                                const std::vector<std::string>& robot_part_names,
                                                                                const double sampling_frequency)
  {
    // learn cartesian space dmp
    typename DMPType::DMPPtr cartesian_dmp;
    ROS_VERIFY(learnCartesianSpaceDMP(cartesian_dmp, node_handle, abs_bag_file_name, robot_part_names, sampling_frequency));

    // learn joint space dmp
    typename DMPType::DMPPtr joint_dmp;

    ROS_VERIFY(learnJointSpaceDMP(joint_dmp, node_handle, abs_bag_file_name,
                                  robot_part_names, sampling_frequency));

    // add the two dmps
    dmp = cartesian_dmp;
    ROS_VERIFY(dmp->add(*joint_dmp, false));
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnCartesianAndJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                                ros::NodeHandle& node_handle,
                                                                                const std::string& abs_bag_file_name,
                                                                                const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                                const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                                const Eigen::VectorXd& start,
                                                                                const Eigen::VectorXd& goal,
                                                                                const double sampling_frequency)
  {
    // learn cartesian space dmp
    typename DMPType::DMPPtr cartesian_dmp;
    ROS_VERIFY(learnCartesianSpaceDMP(cartesian_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // learn minimum jerk dmp
    double initial_duration = 0;
    ROS_VERIFY(cartesian_dmp->getInitialDuration(initial_duration));
    typename DMPType::DMPPtr min_jerk_dmp;
    dynamic_movement_primitive::TypeMsg type;
    type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE;
    ROS_VERIFY(learnDMPFromMinJerk(min_jerk_dmp, node_handle, type, robot_part_names_from_min_jerk, initial_duration, start, goal, sampling_frequency));

    // learn joint space dmp
    typename DMPType::DMPPtr joint_dmp;
    ROS_VERIFY(learnJointSpaceDMP(joint_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // add the three dmps
    dmp = cartesian_dmp;
    ROS_VERIFY(dmp->add(*min_jerk_dmp, false));
    ROS_VERIFY(dmp->add(*joint_dmp, false));
    return true;
  }

template<class DMPType>
  bool DynamicMovementPrimitiveLearner<DMPType>::learnCartesianAndJointSpaceDMP(typename DMPType::DMPPtr& dmp,
                                                                                ros::NodeHandle& node_handle,
                                                                                const std::string& abs_bag_file_name,
                                                                                const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                                const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                                const std::vector<Eigen::VectorXd>& waypoints,
                                                                                const std::vector<double>& duration_fractions,
                                                                                const double sampling_frequency)
  {
    // learn cartesian space dmp
    typename DMPType::DMPPtr cartesian_dmp;
    ROS_VERIFY(learnCartesianSpaceDMP(cartesian_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // learn minimum jerk dmp
    double initial_duration = 0;
    ROS_VERIFY(cartesian_dmp->getInitialDuration(initial_duration));

    std::vector<double> durations;
    ROS_VERIFY(TrajectoryUtilities::getDurations(initial_duration, duration_fractions, durations));

    typename DMPType::DMPPtr min_jerk_dmp;
    dynamic_movement_primitive::TypeMsg type;
    type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE;
    ROS_VERIFY(learnDMPFromMinJerk(min_jerk_dmp, node_handle, type, robot_part_names_from_min_jerk, durations, waypoints, sampling_frequency));

    // learn joint space dmp
    typename DMPType::DMPPtr joint_dmp;
    ROS_VERIFY(learnJointSpaceDMP(joint_dmp, node_handle, abs_bag_file_name, robot_part_names_from_trajectory, sampling_frequency));

    // add the three dmps
    dmp = cartesian_dmp;
    ROS_VERIFY(dmp->add(*min_jerk_dmp, false));
    ROS_VERIFY(dmp->add(*joint_dmp, false));
    return true;
  }
}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_LEARNER_H_ */
