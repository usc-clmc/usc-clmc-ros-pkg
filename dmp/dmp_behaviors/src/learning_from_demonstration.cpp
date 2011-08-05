/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   learning_from_demonstration.cpp

  \author Peter Pastor
  \date   Jan 23, 2011

 *********************************************************************/

// system includes
#include <ros/package.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <skill_library/addAffordance.h>
#include <skill_library/Affordance.h>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

// local includes
#include <dmp_behaviors/learning_from_demonstration.h>
#include <dmp_behaviors/behavior_utilities.h>

using namespace dmp_behavior_actions;
using namespace skill_library;

namespace dmp_behaviors
{

LearningFromDemonstration::LearningFromDemonstration(ros::NodeHandle& node_handle, const std::string& action_name) :
        node_handle_(node_handle),
        action_server_(ros::NodeHandle("/Behaviors"), action_name,
                       boost::bind(&LearningFromDemonstration::execute, this, _1), false)
{
  ROS_VERIFY(dmp_learner_utilities_.initialize(node_handle_));
}

void LearningFromDemonstration::start()
{
  action_server_.start();
}

void LearningFromDemonstration::execute(const dmp_behavior_actions::LearningFromDemonstrationGoalConstPtr& goal)
{
  ROS_VERIFY(readParams());

  std::vector<std::vector<double> > waypoints;
  for (int i = 0; i < (int)goal->waypoints.size(); ++i)
  {
    waypoints.push_back(goal->waypoints[i].waypoint);
  }

  bool service_online = false;
  while (ros::ok() && !service_online)
  {
    std::string service_name = "/SkillLibrary/addAffordance";
    add_affordance_service_client_ = node_handle_.serviceClient<skill_library::addAffordance> (service_name);
    if (!add_affordance_service_client_.waitForExistence(ros::Duration(1.0)))
    {
      ROS_WARN("Waiting for >%s< ...", service_name.c_str());
    }
    else
    {
      service_online = true;
    }
  }

  if(!goal->robot_part_names_from_trajectory.empty() && goal->joint_states_bag_file_name.empty())
  {
    BehaviorUtilities<LearningFromDemonstration, ActionServer>::failed("Joint states bag file name is empty.", action_server_);
    return;
  }
  std::string absolute_file_name;
  dynamic_movement_primitive::DMPUtilitiesMsg dmp_utilities_msg;
  if(goal->robot_part_names_from_min_jerk.empty())
  {
    absolute_file_name = demonstrations_directory_path_ + goal->joint_states_bag_file_name;
    ROS_INFO("Learning DMP from trajecotry stored in file >%s<.", absolute_file_name.c_str());
    if (!dmp_learner_utilities_.learnDMPsFromTrajectory(absolute_file_name, goal->type, goal->robot_part_names_from_trajectory, dmp_utilities_msg))
    {
      BehaviorUtilities<LearningFromDemonstration, ActionServer>::failed("Could not learn DMPs from trajectory.", action_server_);
      return;
    }
  }
  else if(goal->robot_part_names_from_trajectory.empty())
  {
    ROS_INFO("Learning DMP from minimum jerk.");
    for(int i=0; i<(int)goal->durations.size(); ++i)
    {
      if(goal->durations[i] < 0.5)
      {
        std::stringstream ss;
        ss << goal->durations[i];
        BehaviorUtilities<LearningFromDemonstration, ActionServer>::failed("Initial duration of >" + ss.str() + "< seconds is too short. This is just for safety.", action_server_);
        return;
      }
    }
    if (!dmp_learner_utilities_.learnDMPsFromMinJerk(goal->type, goal->robot_part_names_from_min_jerk, goal->durations, waypoints, dmp_utilities_msg))
    {
      BehaviorUtilities<LearningFromDemonstration, ActionServer>::failed("Could not learn DMPs from trajectory and minimum jerk.", action_server_);
      return;
    }
  }
  else
  {
    absolute_file_name = demonstrations_directory_path_ + goal->joint_states_bag_file_name;
    ROS_INFO("Learning DMP from trajectory stored in >%s< and from a generated minimum jerk.", absolute_file_name.c_str());
    if (!dmp_learner_utilities_.learnDMPsFromTrajectoryAndMinJerk(absolute_file_name, goal->type, goal->robot_part_names_from_trajectory,
                                                                  goal->robot_part_names_from_min_jerk, goal->durations, waypoints, dmp_utilities_msg))
    {
      BehaviorUtilities<LearningFromDemonstration, ActionServer>::failed("Could not learn DMPs from trajectory and minimum jerk.", action_server_);
      return;
    }
  }

  // TODO: change this...
  dmp_utilities_msg.dmp_version = dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009;

  skill_library::Affordance affordance;
  affordance.dmp = dmp_utilities_msg;
  affordance.object = goal->object;
  affordance.task = goal->task;

  addAffordance add_affordance_service;
  add_affordance_service.request.affordance = affordance;
  if(!add_affordance_service_client_.call(add_affordance_service))
  {
    BehaviorUtilities<LearningFromDemonstration, ActionServer>::failed("Could not add affordance to skill library.", action_server_);
    return;
  }

  LearningFromDemonstrationResult result;
  result.affordance = affordance;
  result.result = LearningFromDemonstrationResult::SUCCEEDED;
  action_server_.setSucceeded(result);
}

bool LearningFromDemonstration::readParams()
{
  std::string demonstration_data_directory;
  ROS_ASSERT(usc_utilities::read(node_handle_, "demonstration_data_directory", demonstration_data_directory));
  usc_utilities::appendTrailingSlash(demonstration_data_directory);

  std::string package_name;
  ROS_ASSERT(usc_utilities::read(node_handle_, "package_name", package_name));
  std::string absolute_path = ros::package::getPath(package_name);
  usc_utilities::appendTrailingSlash(absolute_path);

  demonstrations_directory_path_ = absolute_path + demonstration_data_directory;
  return true;
}

}
