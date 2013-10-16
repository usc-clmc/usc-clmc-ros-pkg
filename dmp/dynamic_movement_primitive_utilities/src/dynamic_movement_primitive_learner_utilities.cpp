/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_learner_utilities.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 27, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_learner_utilities.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

using namespace std;
using namespace dmp;

namespace dmp_utilities
{

bool DynamicMovementPrimitiveLearnerUtilities::initialize(ros::NodeHandle& node_handle)
{
  node_handle_ = node_handle;
  return (initialized_ = true);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromTrajectory(const std::string& abs_bag_file_name,
                                                                       const dynamic_movement_primitive::TypeMsg& type,
                                                                       const std::vector<std::string>& robot_part_names,
                                                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);

  if(type.type == type.DISCRETE_JOINT_SPACE)
  {
    if(!learnJointSpaceDMPs(abs_bag_file_name, robot_part_names, dmp_utilities_msg))
    {
      ROS_ERROR("Could not learn joint space DMP.");
      return false;
    }
  }
  else if(type.type == type.DISCRETE_CARTESIAN_SPACE)
  {
    if(!learnCartesianSpaceDMPs(abs_bag_file_name, robot_part_names, dmp_utilities_msg))
    {
      ROS_ERROR("Could not learn pure cartesian space trajectory");
      return false;
    }
  }
  else if(type.type == type.DISCRETE_CARTESIAN_AND_JOINT_SPACE)
  {
    if(!learnCartesianAndJointSpaceDMPs(abs_bag_file_name, robot_part_names, dmp_utilities_msg))
    {
      ROS_ERROR("Could not learn cartesian and joint space DMP.");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Unknown DMP type requested >%i<.", type.type);
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                                                                    const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                    const double initial_duration,
                                                                    const Eigen::VectorXd& start,
                                                                    const Eigen::VectorXd& goal,
                                                                    dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<double> initial_durations;
  initial_durations.push_back(initial_duration);
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);
  return learnDMPsFromMinJerk(type, robot_part_names_from_min_jerk, initial_durations, waypoints, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                                                                    const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                    const double initial_duration,
                                                                    const std::vector<double>& start,
                                                                    const std::vector<double>& goal,
                                                                    dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  return learnDMPsFromMinJerk(type, robot_part_names_from_min_jerk, initial_duration, Eigen::VectorXd::Map(&start[0], start.size()), Eigen::VectorXd::Map(&goal[0], goal.size()), dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                                                                    const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                    const std::vector<double>& initial_durations,
                                                                    const std::vector<Eigen::VectorXd>& waypoints,
                                                                    dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{

  ROS_ASSERT(initialized_);

  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  if(type.type == dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE)
  {
    dmp_lib::ICRA2009DMPPtr icra2009_dmp_cartesian;
    dynamic_movement_primitive::TypeMsg tmp_type;
    tmp_type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE;
    ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnDMPFromMinJerk(icra2009_dmp_cartesian, node_handle_, tmp_type, robot_part_names_from_min_jerk, initial_durations, waypoints));

    dmp_lib::ICRA2009DMPPtr icra2009_dmp_joint;
    tmp_type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE;
    ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnDMPFromMinJerk(icra2009_dmp_joint, node_handle_, tmp_type, robot_part_names_from_min_jerk, initial_durations, waypoints));

    icra2009_dmp = icra2009_dmp_cartesian;
    ROS_VERIFY(icra2009_dmp->add(*icra2009_dmp_joint));
  }
  else
  {
    ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnDMPFromMinJerk(icra2009_dmp, node_handle_, type, robot_part_names_from_min_jerk, initial_durations, waypoints));
  }

  ICRA2009DMP::DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
  dmp_utilities_msg.icra2009_dmp = icra2009_dmp_msg;

  //  dmp_lib::NC2010DMPPtr nc2010_dmp;
  //  if(type.type == dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE)
  //  {
  //    dmp_lib::NC2010DMPPtr nc2010_dmp_cartesian;
  //    dynamic_movement_primitive::TypeMsg tmp_type;
  //    tmp_type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE;
  //    ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnDMPFromMinJerk(nc2010_dmp_cartesian, node_handle_, tmp_type, robot_part_names_from_min_jerk, initial_durations, waypoints));
  //
  //    dmp_lib::NC2010DMPPtr nc2010_dmp_joint;
  //    tmp_type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE;
  //    ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnDMPFromMinJerk(nc2010_dmp_joint, node_handle_, tmp_type, robot_part_names_from_min_jerk, initial_durations, waypoints));
  //
  //    nc2010_dmp = nc2010_dmp_cartesian;
  //    ROS_VERIFY(nc2010_dmp->add(*nc2010_dmp_joint));
  //  }
  //  else
  //  {
  //    ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnDMPFromMinJerk(nc2010_dmp, node_handle_, type, robot_part_names_from_min_jerk, initial_durations, waypoints));
  //  }
  //
  //  NC2010DMP::DMPMsg nc2010_dmp_msg;
  //  ROS_VERIFY(NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
  //  dmp_utilities_msg.nc2010_dmp = nc2010_dmp_msg;

  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromMinJerk(const dynamic_movement_primitive::TypeMsg& type,
                          const std::vector<std::string>& robot_part_names_from_min_jerk,
                          const std::vector<double>& initial_durations,
                          const std::vector<std::vector<double> >& waypoints,
                          dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<Eigen::VectorXd> eigen_waypoints;
  for (int i = 0; i < (int)waypoints.size(); ++i)
  {
    Eigen::VectorXd waypoint = Eigen::VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return learnDMPsFromMinJerk(type, robot_part_names_from_min_jerk, initial_durations, eigen_waypoints, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                                                                 const dynamic_movement_primitive::TypeMsg& type,
                                                                                 const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                                 const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                                 const Eigen::VectorXd& start,
                                                                                 const Eigen::VectorXd& goal,
                                                                                 dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<double> duration_fractions;
  duration_fractions.push_back(1.0);
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);
  return learnDMPsFromTrajectoryAndMinJerk(abs_bag_file_name, type, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, duration_fractions, waypoints, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                                                                 const dynamic_movement_primitive::TypeMsg& type,
                                                                                 const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                                 const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                                 const std::vector<double>& start,
                                                                                 const std::vector<double>& goal,
                                                                                 dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  return learnDMPsFromTrajectoryAndMinJerk(abs_bag_file_name, type, robot_part_names_from_trajectory, robot_part_names_from_min_jerk,
                                           Eigen::VectorXd::Map(&start[0], start.size()), Eigen::VectorXd::Map(&goal[0], goal.size()), dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                                                                 const dynamic_movement_primitive::TypeMsg& type,
                                                                                 const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                                 const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                                 const std::vector<double>& duration_fractions,
                                                                                 const std::vector<Eigen::VectorXd>& waypoints,
                                                                                 dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);
  if(type.type == type.DISCRETE_JOINT_SPACE)
  {
    if(!learnJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions, dmp_utilities_msg))
    {
      ROS_ERROR("Could not learn joint space DMP.");
      return false;
    }
  }
  else if(type.type == type.DISCRETE_CARTESIAN_SPACE)
  {
    if(!learnCartesianSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions, dmp_utilities_msg))
    {
      ROS_ERROR("Could not learn pure cartesian space trajectory");
      return false;
    }
  }
  else if(type.type == type.DISCRETE_CARTESIAN_AND_JOINT_SPACE)
  {
    if(!learnCartesianAndJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions, dmp_utilities_msg))
    {
      ROS_ERROR("Could not learn cartesian and joint space DMP.");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Unknown DMP type requested >%i<.", type.type);
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnDMPsFromTrajectoryAndMinJerk(const std::string& abs_bag_file_name,
                                                                                 const dynamic_movement_primitive::TypeMsg& type,
                                                                                 const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                                 const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                                 const std::vector<double>& duration_fractions,
                                                                                 const std::vector<std::vector<double> >& waypoints,
                                                                                 dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<Eigen::VectorXd> eigen_waypoints;
  for (int i = 0; i < (int)waypoints.size(); ++i)
  {
    Eigen::VectorXd waypoint = Eigen::VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return learnDMPsFromTrajectoryAndMinJerk(abs_bag_file_name, type, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, duration_fractions, eigen_waypoints, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                   const std::vector<std::string>& robot_part_names,
                                                                   dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);

  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnJointSpaceDMP(icra2009_dmp, node_handle_, abs_bag_file_name, robot_part_names));

  ICRA2009DMP::DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
  dmp_utilities_msg.icra2009_dmp = icra2009_dmp_msg;

  //  dmp_lib::NC2010DMPPtr nc2010_dmp;
  //  ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnJointSpaceDMP(nc2010_dmp, node_handle_, abs_bag_file_name, robot_part_names));
  //
  //  NC2010DMP::DMPMsg nc2010_dmp_msg;
  //  ROS_VERIFY(NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
  //  dmp_utilities_msg.nc2010_dmp = nc2010_dmp_msg;

  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                   const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                   const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                   const Eigen::VectorXd& start,
                                                                   const Eigen::VectorXd& goal,
                                                                   dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<double> duration_fractions;
  duration_fractions.push_back(1.0);
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);
  return learnJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions, dmp_utilities_msg);
}
bool DynamicMovementPrimitiveLearnerUtilities::learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                   const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                   const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                   const std::vector<double>& start,
                                                                   const std::vector<double>& goal,
                                                                   dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  return learnJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk,
                             Eigen::VectorXd::Map(&start[0], start.size()), Eigen::VectorXd::Map(&goal[0], goal.size()), dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                   const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                   const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                   const std::vector<Eigen::VectorXd>& waypoints,
                                                                   const std::vector<double>& duration_fractions,
                                                                   dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);

  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnJointSpaceDMP(icra2009_dmp, node_handle_, abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions));

  ICRA2009DMP::DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
  dmp_utilities_msg.icra2009_dmp = icra2009_dmp_msg;

  //  dmp_lib::NC2010DMPPtr nc2010_dmp;
  //  ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnJointSpaceDMP(nc2010_dmp, node_handle_, abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions));
  //
  //  NC2010DMP::DMPMsg nc2010_dmp_msg;
  //  ROS_VERIFY(NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
  //  dmp_utilities_msg.nc2010_dmp = nc2010_dmp_msg;

  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                   const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                   const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                   const std::vector<std::vector<double> >& waypoints,
                                                                   const std::vector<double>& duration_fractions,
                                                                   dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<Eigen::VectorXd> eigen_waypoints;
  for (int i = 0; i < (int)waypoints.size(); ++i)
  {
    Eigen::VectorXd waypoint = Eigen::VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return learnJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, eigen_waypoints, duration_fractions, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                                                                       const std::vector<std::string>& robot_part_names,
                                                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);

  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnCartesianSpaceDMP(icra2009_dmp, node_handle_, abs_bag_file_name, robot_part_names));

  ICRA2009DMP::DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
  dmp_utilities_msg.icra2009_dmp = icra2009_dmp_msg;

  //  dmp_lib::NC2010DMPPtr nc2010_dmp;
  //  ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnCartesianSpaceDMP(nc2010_dmp, node_handle_, abs_bag_file_name, robot_part_names));
  //
  //  NC2010DMP::DMPMsg nc2010_dmp_msg;
  //  ROS_VERIFY(NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
  //  dmp_utilities_msg.nc2010_dmp = nc2010_dmp_msg;

  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                                                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                       const Eigen::VectorXd& start,
                                                                       const Eigen::VectorXd& goal,
                                                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{

  std::vector<double> duration_fractions;
  duration_fractions.push_back(1.0);
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);
  return learnCartesianSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                                                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                       const std::vector<double>& start,
                                                                       const std::vector<double>& goal,
                                                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  return learnCartesianSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk,
                                 Eigen::VectorXd::Map(&start[0], start.size()), Eigen::VectorXd::Map(&goal[0], goal.size()), dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                                                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                       const std::vector<Eigen::VectorXd>& waypoints,
                                                                       const std::vector<double>& duration_fractions,
                                                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);
  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnCartesianSpaceDMP(icra2009_dmp, node_handle_, abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions));

  ICRA2009DMP::DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
  dmp_utilities_msg.icra2009_dmp = icra2009_dmp_msg;

  //  dmp_lib::NC2010DMPPtr nc2010_dmp;
  //  ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnCartesianSpaceDMP(nc2010_dmp, node_handle_, abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions));
  //
  //  NC2010DMP::DMPMsg nc2010_dmp_msg;
  //  ROS_VERIFY(NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
  //  dmp_utilities_msg.nc2010_dmp = nc2010_dmp_msg;
  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianSpaceDMPs(const std::string& abs_bag_file_name,
                                                                       const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                       const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                       const std::vector<std::vector<double> >& waypoints,
                                                                       const std::vector<double>& duration_fractions,
                                                                       dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<Eigen::VectorXd> eigen_waypoints;
  for (int i = 0; i < (int)waypoints.size(); ++i)
  {
    Eigen::VectorXd waypoint = Eigen::VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return learnCartesianSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, eigen_waypoints, duration_fractions, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                               const std::vector<std::string>& robot_part_names,
                                                                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);

  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnCartesianAndJointSpaceDMP(icra2009_dmp, node_handle_, abs_bag_file_name, robot_part_names));

  ICRA2009DMP::DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
  dmp_utilities_msg.icra2009_dmp = icra2009_dmp_msg;

  //  dmp_lib::NC2010DMPPtr nc2010_dmp;
  //  ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnCartesianAndJointSpaceDMP(nc2010_dmp, node_handle_, abs_bag_file_name, robot_part_names));
  //
  //  NC2010DMP::DMPMsg nc2010_dmp_msg;
  //  ROS_VERIFY(NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
  //  dmp_utilities_msg.nc2010_dmp = nc2010_dmp_msg;

  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                               const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                               const Eigen::VectorXd& start,
                                                                               const Eigen::VectorXd& goal,
                                                                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<double> duration_fractions;
  duration_fractions.push_back(1.0);
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);
  return learnCartesianAndJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions, dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                               const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                               const std::vector<double>& start,
                                                                               const std::vector<double>& goal,
                                                                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  return learnCartesianAndJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk,
                                         Eigen::VectorXd::Map(&start[0], start.size()), Eigen::VectorXd::Map(&goal[0], goal.size()), dmp_utilities_msg);
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                               const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                               const std::vector<Eigen::VectorXd>& waypoints,
                                                                               const std::vector<double>& duration_fractions,
                                                                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  ROS_ASSERT(initialized_);

  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  ROS_VERIFY(DynamicMovementPrimitiveLearner<ICRA2009DMP>::learnCartesianAndJointSpaceDMP(icra2009_dmp, node_handle_, abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions));

  ICRA2009DMP::DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
  dmp_utilities_msg.icra2009_dmp = icra2009_dmp_msg;

  //  dmp_lib::NC2010DMPPtr nc2010_dmp;
  //  ROS_VERIFY(DynamicMovementPrimitiveLearner<NC2010DMP>::learnCartesianAndJointSpaceDMP(nc2010_dmp, node_handle_, abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, waypoints, duration_fractions));
  //
  //  NC2010DMP::DMPMsg nc2010_dmp_msg;
  //  ROS_VERIFY(NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
  //  dmp_utilities_msg.nc2010_dmp = nc2010_dmp_msg;

  return true;
}

bool DynamicMovementPrimitiveLearnerUtilities::learnCartesianAndJointSpaceDMPs(const std::string& abs_bag_file_name,
                                                                               const std::vector<std::string>& robot_part_names_from_trajectory,
                                                                               const std::vector<std::string>& robot_part_names_from_min_jerk,
                                                                               const std::vector<std::vector<double> >& waypoints,
                                                                               const std::vector<double>& duration_fractions,
                                                                               dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg)
{
  std::vector<Eigen::VectorXd> eigen_waypoints;
  for (int i = 0; i < (int)waypoints.size(); ++i)
  {
    Eigen::VectorXd waypoint = Eigen::VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return learnCartesianAndJointSpaceDMPs(abs_bag_file_name, robot_part_names_from_trajectory, robot_part_names_from_min_jerk, eigen_waypoints, duration_fractions, dmp_utilities_msg);
}

}

