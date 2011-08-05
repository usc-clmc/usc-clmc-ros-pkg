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
     * @param start_link_name
     * @param end_link_name
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

  private:

    /*! Constructor
     */
    DynamicMovementPrimitiveLearner() {};

    /*! Destructor
     */
    virtual ~DynamicMovementPrimitiveLearner() {};

    /*!
     * @param initial_duration
     * @param duration_fractions
     * @param durations
     * @return True on success, otherwise false
     */
    static bool getDurations(const double initial_duration,
                             const std::vector<double>& duration_fractions,
                             std::vector<double>& durations);

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
    if (robot_info::RobotInfo::containsWrenchParts(robot_part_names))
    {
      dmp_lib::Trajectory wrench_trajectory;
      std::vector<std::string> dmp_wrench_variable_names = dmp->getVariableNames();
      robot_info::RobotInfo::extractWrenchNames(dmp_wrench_variable_names);
      // TODO: change the topic name appropriately
      ROS_VERIFY(TrajectoryUtilities::createWrenchTrajectory(wrench_trajectory, dmp_wrench_variable_names, abs_bag_file_name, sampling_frequency, "/SL/right_arm_wrench_processed"));
      if (trajectory.isInitialized())
      {
        ROS_VERIFY(trajectory.cutAndCombine(wrench_trajectory));
      }
      else
      {
        trajectory = wrench_trajectory;
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
  bool DynamicMovementPrimitiveLearner<DMPType>::getDurations(const double initial_duration,
                                                              const std::vector<double>& duration_fractions,
                                                              std::vector<double>& durations)
  {
    // error checking
    double beginning = 0.0;
    for (int i = 0; i < (int)duration_fractions.size(); ++i)
    {
      if (beginning > duration_fractions[i])
      {
        ROS_ERROR("Duration fractions must monotonically increase.");
        return false;
      }
      beginning = duration_fractions[i];
    }
    if(!(beginning < 1.0))
    {
      ROS_ERROR("Duration fractions must be strictly less then 1.0.");
      return false;
    }

    durations.clear();
    if (duration_fractions.empty())
    {
      durations.push_back(initial_duration);
      return true;
    }
    else
    {
      durations.push_back(duration_fractions[0] * initial_duration);
      for (int i = 1; i < (int)duration_fractions.size(); ++i)
      {
        durations.push_back((duration_fractions[i]-duration_fractions[i-1]) * initial_duration);
      }
      durations.push_back((1.0 - duration_fractions[duration_fractions.size()-1]) * initial_duration);
    }

    double total_duration = 0.0;
    for (int i = 0; i < (int)durations.size(); ++i)
    {
      total_duration += durations[i];
    }

    return (fabs(initial_duration - total_duration) < 10e-6);
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
    ROS_VERIFY(getDurations(initial_duration, duration_fractions, durations));

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
    ROS_DEBUG("Learning cartesian space DMP from file >%s< with the following robot parts:", abs_bag_file_name.c_str());
    for (int i = 0; i < (int)robot_part_names.size(); ++i)
    {
      ROS_DEBUG("- >%s<", robot_part_names[i].c_str());
    }

    std::string base_link_name;
    ROS_VERIFY(usc_utilities::read(node_handle, "base_link_name", base_link_name));
    ros::NodeHandle cartesian_space_node_handle(node_handle, "cartesian_space_dmp");

    for (int i = 0; i < (int)robot_part_names.size(); ++i)
    {
      ROS_INFO("Learning cartesian space DMP for >%s<.", robot_part_names[i].c_str());
      ros::NodeHandle robot_part_node_handle(cartesian_space_node_handle, robot_part_names[i]);

      bool first = true;
      typename DMPType::DMPPtr tmp_dmp;
      std::string end_link_name;
      if(robot_part_node_handle.getParam("end_link_name", end_link_name))
      {
        // TODO: Check this !!!
        std::vector<std::string> robot_part_names_for_cartesian_space;
        robot_part_names_for_cartesian_space.push_back(robot_part_names[i]);
        // TODO: Check this !!!

        // initialize dmp from node handle
        ROS_VERIFY(DMPType::initFromNodeHandle(tmp_dmp, robot_part_names_for_cartesian_space, cartesian_space_node_handle));

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

        dmp_lib::Trajectory pose_trajectory;
        ROS_VERIFY(TrajectoryUtilities::createPoseTrajectory(pose_trajectory, joint_trajectory, base_link_name, end_link_name, tmp_dmp->getVariableNames()));
        ROS_VERIFY(pose_trajectory.computeDerivatives());

        // learn dmp
        ROS_VERIFY(tmp_dmp->learnFromTrajectory(pose_trajectory));
        tmp_dmp->changeType(dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE);

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
        ROS_WARN("Skipping learning cartesian space DMP for robot part >%s<.", robot_part_names[i].c_str());
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
    ROS_VERIFY(getDurations(initial_duration, duration_fractions, durations));

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
    ROS_VERIFY(learnJointSpaceDMP(joint_dmp, node_handle, abs_bag_file_name, robot_part_names, sampling_frequency));

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
    ROS_VERIFY(getDurations(initial_duration, duration_fractions, durations));

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
