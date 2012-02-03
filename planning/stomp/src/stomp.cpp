/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

// system includes
#include <cassert>

// ros includes
#include <ros/package.h>
#include <stomp/stomp.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <boost/filesystem.hpp>

namespace stomp
{

STOMP::STOMP()
    : initialized_(false), policy_iteration_counter_(0)
{
}

STOMP::~STOMP()
{
}

/*
bool PolicyImprovementLoop::initializeAndRunTaskByName(ros::NodeHandle& node_handle, std::string& task_name)
{
    // load the task
    TaskManager task_manager;
    ROS_VERIFY(task_manager.initialize());
    boost::shared_ptr<Task> task;
    ROS_VERIFY(task_manager.getTaskByName(task_name, task));

    // initialize the PI loop
    ROS_VERIFY(initialize(node_handle, task));

    int first_trial, last_trial;

    // first_trial defaults to 1:
    node_handle.param("first_trial", first_trial, 1);
    //ROS_VERIFY(stomp_motion_planner::read(node_handle, std::string("first_trial"), first_trial));
    ROS_VERIFY(stomp_motion_planner::read(node_handle, std::string("last_trial"), last_trial));

    for (int i=first_trial; i<=last_trial; ++i)
    {
        ROS_VERIFY(runSingleIteration(i));
        ros::spinOnce();
    }
    return true;
}*/

bool STOMP::initialize(ros::NodeHandle& node_handle, boost::shared_ptr<stomp::Task> task)
{
    node_handle_ = node_handle;
    ROS_VERIFY(readParameters());

    task_ = task;
    ROS_VERIFY(task_->getPolicy(policy_));
    ROS_VERIFY(policy_->getNumTimeSteps(num_time_steps_));
    ROS_VERIFY(task_->getControlCostWeight(control_cost_weight_));

    ROS_VERIFY(policy_->getNumDimensions(num_dimensions_));
    ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_decay_.size()));
    ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_stddev_.size()));
//    ROS_INFO("Learning policy with %i dimensions.", num_dimensions_);

    policy_improvement_.initialize(num_rollouts_, num_time_steps_, num_reused_rollouts_, 1, policy_, use_cumulative_costs_);

    tmp_rollout_cost_ = Eigen::VectorXd::Zero(num_time_steps_);
    rollout_costs_ = Eigen::MatrixXd::Zero(num_rollouts_, num_time_steps_);
    time_step_weights_.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));

    policy_iteration_counter_ = 0;
    return (initialized_ = true);
}

bool STOMP::readParameters()
{
    ROS_VERIFY(usc_utilities::read(node_handle_, std::string("num_rollouts"), num_rollouts_));
    ROS_VERIFY(usc_utilities::read(node_handle_, std::string("num_reused_rollouts"), num_reused_rollouts_));
    //ROS_VERIFY(usc_utilities::read(node_handle_, std::string("num_time_steps"), num_time_steps_));

    ROS_VERIFY(usc_utilities::readDoubleArray(node_handle_, "noise_stddev", noise_stddev_));
    ROS_VERIFY(usc_utilities::readDoubleArray(node_handle_, "noise_decay", noise_decay_));
    node_handle_.param("write_to_file", write_to_file_, true); // defaults are sometimes good!
    node_handle_.param("use_cumulative_costs", use_cumulative_costs_, false);
    return true;
}

bool STOMP::readPolicy(const int iteration_number)
{
    // check whether reading the policy from file is neccessary
    if(iteration_number == (policy_iteration_counter_))
    {
        return true;
    }
/*    ROS_INFO("Read policy from file %s.", policy_->getFileName(iteration_number).c_str());
    ROS_VERIFY(policy_->readFromDisc(policy_->getFileName(iteration_number)));
    ROS_VERIFY(task_->setPolicy(policy_));
*/    return true;
}

bool STOMP::writePolicy(const int iteration_number, bool is_rollout, int rollout_id)
{
    return true;
}

void STOMP::clearReusedRollouts()
{
  policy_improvement_.clearReusedRollouts();
}

bool STOMP::runSingleIteration(const int iteration_number)
{
    ROS_ASSERT(initialized_);
    policy_iteration_counter_++;

    if (write_to_file_)
    {
        // load new policy if neccessary
        ROS_VERIFY(readPolicy(iteration_number));
    }

    // compute appropriate noise values
    std::vector<double> noise;
    noise.resize(num_dimensions_);
    for (int i=0; i<num_dimensions_; ++i)
    {
        noise[i] = noise_stddev_[i] * pow(noise_decay_[i], iteration_number-1);
    }

    // get rollouts and execute them
    ROS_VERIFY(policy_improvement_.getRollouts(rollouts_, noise));

    for (int r=0; r<int(rollouts_.size()); ++r)
    {
        ROS_VERIFY(task_->execute(rollouts_[r], tmp_rollout_cost_, tmp_rollout_weighted_features_, iteration_number));
        rollout_costs_.row(r) = tmp_rollout_cost_.transpose();
        //ROS_INFO("Rollout %d, cost = %lf", r+1, tmp_rollout_cost_.sum());
    }

    // TODO: fix this std::vector<>
    std::vector<double> all_costs;
    ROS_VERIFY(policy_improvement_.setRolloutCosts(rollout_costs_, control_cost_weight_, all_costs));

    // improve the policy
    ROS_VERIFY(policy_improvement_.improvePolicy(parameter_updates_));
    ROS_VERIFY(policy_improvement_.getTimeStepWeights(time_step_weights_));
    ROS_VERIFY(policy_->updateParameters(parameter_updates_, time_step_weights_));

    // get a noise-less rollout to check the cost
    ROS_VERIFY(policy_->getParameters(parameters_));
    ROS_VERIFY(task_->execute(parameters_, tmp_rollout_cost_, tmp_rollout_weighted_features_, iteration_number));
    ROS_INFO("Noiseless cost = %lf", tmp_rollout_cost_.sum());

    // add the noiseless rollout into policy_improvement:
    std::vector<std::vector<Eigen::VectorXd> > extra_rollout;
    std::vector<Eigen::VectorXd> extra_rollout_cost;
    extra_rollout.resize(1);
    extra_rollout_cost.resize(1);
    extra_rollout[0] = parameters_;
    extra_rollout_cost[0] = tmp_rollout_cost_;
    ROS_VERIFY(policy_improvement_.addExtraRollouts(extra_rollout, extra_rollout_cost));

    if (write_to_file_)
    {
        // store updated policy to disc
        //ROS_VERIFY(writePolicy(iteration_number));
        //ROS_VERIFY(writePolicyImprovementStatistics(stats_msg));
    }

    return true;
}

/*
bool PolicyImprovementLoop::writePolicyImprovementStatistics(const policy_improvement_loop::PolicyImprovementStatistics& stats_msg)
{

    std::string directory_name = std::string("/tmp/pi2_statistics/");
    std::string file_name = directory_name;
    file_name.append(std::string("pi2_statistics.bag"));

    if (!boost::filesystem::exists(directory_name))
    {
        if(stats_msg.iteration == 1)
        {
            boost::filesystem::remove_all(directory_name);
        }
        ROS_INFO("Creating directory %s...", directory_name.c_str());
        ROS_VERIFY(boost::filesystem::create_directories(directory_name));
    }

    try
    {
        if(stats_msg.iteration == 1)
        {
            rosbag::Bag bag(file_name, rosbag::bagmode::Write);
            bag.write(PI_STATISTICS_TOPIC_NAME, ros::Time::now(), stats_msg);
            bag.close();
        }
        else
        {
            rosbag::Bag bag(file_name, rosbag::bagmode::Append);
            bag.write(PI_STATISTICS_TOPIC_NAME, ros::Time::now(), stats_msg);
            bag.close();
        }
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could write to bag file %s: %s", file_name.c_str(), ex.what());
        return false;
    }
    return true;
}
*/
}
