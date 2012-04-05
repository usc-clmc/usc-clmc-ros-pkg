/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks    ...

 \file   dmp_waypoint_task.cpp

 \author Peter Pastor and Mrinal Kalakrishnan
 \date   Feb 20, 2010

 *********************************************************************/

// system includes
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <gtest/gtest.h>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

// local includes
#include "dmp_waypoint_task.h"

using namespace usc_utilities;
using namespace Eigen;

PLUGINLIB_DECLARE_CLASS(policy_improvement_loop_test, DMPWaypointTask, policy_improvement_loop_test::DMPWaypointTask, task_manager_interface::Task);

namespace policy_improvement_loop_test
{

bool DMPWaypointTask::initialize(ros::NodeHandle& node_handle)
{
  node_handle_ = node_handle;
  readParameters();

  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  ros::NodeHandle dmp_node_handle(node_handle, "dmp");
  dmp::ICRA2009DynamicMovementPrimitive::initFromNodeHandle(icra2009_dmp, dmp_node_handle);

  sampling_frequency_ = static_cast<double>(num_time_steps_) / movement_time_;
  if(!icra2009_dmp->learnFromMinimumJerk(start_, goal_, sampling_frequency_, movement_time_))
  {
    ROS_ERROR("Could initialize DMP from minimum jerk. Cannot initialize task.");
    return false;
  }

  policy_.reset(new policy_library::DMPPolicy());
  policy_->initialize(icra2009_dmp);

  policy_->setNumTimeSteps(num_time_steps_);

  // policy_->setFileNameBase("/tmp/ctp_");
  // policy_->setToMinControlCost(start_, goal_);

  trial_counter_ = 0;
  noiseless_rollout_counter_ = 0;
  noisy_rollout_counter_ = 0;
  logging_flag_ = false;

  return true;
}

bool DMPWaypointTask::execute(std::vector<Eigen::VectorXd>& parameters,
                              Eigen::VectorXd& costs,
                              double& terminal_cost,
                              const int iteration_number)
{
  int waypoint_time_index = int(double(num_time_steps_) * (waypoint_time_ / movement_time_));

  dmp_lib::DMPPtr dmp;
  ROS_VERIFY(policy_->getDMP(dmp));

  if(!dmp->setThetas(parameters))
  {
    ROS_ERROR("Could not set parameters of the DMP.");
    return false;
  }

  if(!dmp->setup())
  {
    ROS_ERROR("Could not setup DMP.");
    return false;
  }

  dmp_lib::Trajectory rollout;
  if(!dmp->propagateFull(rollout, movement_time_, num_time_steps_))
  {
    ROS_ERROR("Could not propagate DMP.");
    return false;
  }

  if(!logging_flag_ && trial_counter_ >= num_rollouts_ && (trial_counter_ % num_reused_rollouts_ == 0))
  {
    logging_flag_ = true;
    std::stringstream ss;
    ss << noiseless_rollout_counter_;
    std::string fname = "/tmp/dmp_waypoint_task_noisless_rollout_" + ss.str() + ".clmc";
    if(!rollout.writeToCLMCFile(fname))
    {
      ROS_ERROR("Could not write rollout to file >%s<.", fname.c_str());
      return false;
    }
    noiseless_rollout_counter_++;
  }
  else
  {
    logging_flag_ = false;
    std::stringstream ss;
    ss << noisy_rollout_counter_;
    std::string fname = "/tmp/dmp_waypoint_task_noisy_rollout_" + ss.str() + ".clmc";
    if(!rollout.writeToCLMCFile(fname))
    {
      ROS_ERROR("Could not write rollout to file >%s<.", fname.c_str());
      return false;
    }
    noisy_rollout_counter_++;
    trial_counter_++;
  }

  if(num_time_steps_ != rollout.getNumContainedSamples())
  {
    ROS_ERROR("Number of time steps >%i< is not the same as the number of trajectory points in the rollout >%i<.",num_time_steps_, rollout.getNumContainedSamples());
    return false;
  }

  costs = VectorXd::Zero(num_time_steps_);
  for (int d = 0; d < num_dimensions_; ++d)
  {

    double position = 0.0;
    if(!rollout.getTrajectoryPosition(waypoint_time_index, d, position))
    {
      ROS_ERROR("Could not get position value from rollout.");
      return false;
    }

    // add cost of missing waypoint target
    double dist = position - waypoint_(d);
    dist *= dist;
    costs[waypoint_time_index] += waypoint_cost_weight_ * dist;

    // add cost of high accelerations
    for (int i = 0; i < rollout.getNumContainedSamples(); ++i)
    {
      double acceleration = 0;
      if(!rollout.getTrajectoryAcceleration(i, d, acceleration))
      {
        ROS_ERROR("Could not get acceleration value from rollout.");
        return false;
      }
      costs[i] += acceleration_cost_weight_ * (acceleration*acceleration);
    }

  }
  return true;
}

bool DMPWaypointTask::getPolicy(boost::shared_ptr<policy_library::Policy>& policy)
{
  policy = policy_;
  return true;
}

bool DMPWaypointTask::setPolicy(const boost::shared_ptr<policy_library::Policy> policy)
{
  policy_ = boost::dynamic_pointer_cast<policy_library::DMPPolicy>(policy);
  return true;
}

bool DMPWaypointTask::getControlCostWeight(double& control_cost_weight)
{
  control_cost_weight = control_cost_weight_;
  return true;
}

void DMPWaypointTask::readParameters()
{
  ROS_VERIFY(readEigenVector(node_handle_, "start", start_));
  ROS_VERIFY(readEigenVector(node_handle_, "goal", goal_));
  ROS_VERIFY(readEigenVector(node_handle_, "waypoint", waypoint_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "waypoint_time", waypoint_time_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "movement_time", movement_time_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "waypoint_cost_weight", waypoint_cost_weight_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "control_cost_weight", control_cost_weight_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "acceleration_cost_weight", acceleration_cost_weight_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "num_dimensions", num_dimensions_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_time_steps", num_time_steps_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "write_to_file", write_to_file_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "num_rollouts", num_rollouts_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_reused_rollouts", num_reused_rollouts_));
}

}
