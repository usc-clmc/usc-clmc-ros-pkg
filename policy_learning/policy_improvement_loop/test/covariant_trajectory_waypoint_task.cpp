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

#include "covariant_trajectory_waypoint_task.h"
#include <usc_utilities/param_server.h>
#include <gtest/gtest.h>

using namespace usc_utilities;
USING_PART_OF_NAMESPACE_EIGEN

PLUGINLIB_DECLARE_CLASS(policy_improvement_loop_test, CovariantTrajectoryWaypointTask,
                        policy_improvement_loop_test::CovariantTrajectoryWaypointTask, task_manager_interface::Task);

namespace policy_improvement_loop_test
{

CovariantTrajectoryWaypointTask::CovariantTrajectoryWaypointTask()
{
}

CovariantTrajectoryWaypointTask::~CovariantTrajectoryWaypointTask()
{
}

bool CovariantTrajectoryWaypointTask::initialize(ros::NodeHandle& node_handle)
{
    node_handle_ = node_handle;
    //num_time_steps_ = 50;

    evaluation_count_ = 0;

    readParameters();

    policy_.reset(new policy_library::CovariantTrajectoryPolicy());
    policy_->initialize(node_handle_);
    //policy_->setFileNameBase("/tmp/ctp_");
    policy_->setToMinControlCost(start_, goal_);
    policy_->getNumTimeSteps(num_time_steps_);
    return true;
}

bool CovariantTrajectoryWaypointTask::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, double& terminal_cost, const int iteration_number)
{
    terminal_cost = 0.0;
    int waypoint_time_index = int(double(num_time_steps_) * (waypoint_time_ / movement_time_));
    costs = VectorXd::Zero(num_time_steps_);
    for (int d=0; d<num_dimensions_; ++d)
    {
        double dist = parameters[d](waypoint_time_index) - waypoint_(d);
        dist *= dist;
        terminal_cost += waypoint_cost_weight_ * dist;
        //costs[waypoint_time_index] += waypoint_cost_weight_ * dist;
    }
    ++evaluation_count_;
    //printf("Evaluations: %d\n", evaluation_count_);
    return true;
}

bool CovariantTrajectoryWaypointTask::getPolicy(boost::shared_ptr<policy_library::Policy>& policy)
{
    policy = policy_;
    return true;
}

bool CovariantTrajectoryWaypointTask::setPolicy(const boost::shared_ptr<policy_library::Policy> policy)
{
    policy_ = boost::dynamic_pointer_cast<policy_library::CovariantTrajectoryPolicy>(policy);
    return true;
}

bool CovariantTrajectoryWaypointTask::getControlCostWeight(double& control_cost_weight)
{
    control_cost_weight = control_cost_weight_;
    return true;
}

void CovariantTrajectoryWaypointTask::readParameters()
{
    EXPECT_TRUE(readEigenVector(node_handle_, "start", start_));
    EXPECT_TRUE(readEigenVector(node_handle_, "goal", goal_));
    EXPECT_TRUE(readEigenVector(node_handle_, "waypoint", waypoint_));
    node_handle_.param("waypoint_time", waypoint_time_, 0.5);
    node_handle_.param("movement_time", movement_time_, 1.0);
    node_handle_.param("waypoint_cost_weight", waypoint_cost_weight_, 1000.0);
    node_handle_.param("control_cost_weight", control_cost_weight_, 0.1);
    node_handle_.param("num_dimensions", num_dimensions_, 1);
}


}
