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

#ifndef COVARIANT_TRAJECTORY_WAYPOINT_TASK_H_
#define COVARIANT_TRAJECTORY_WAYPOINT_TASK_H_

#include <task_manager/task.h>
#include <policy_library/covariant_trajectory_policy.h>

namespace policy_improvement_loop_test
{

class CovariantTrajectoryWaypointTask : public task_manager_interface::Task
{
public:
    CovariantTrajectoryWaypointTask();
    virtual ~CovariantTrajectoryWaypointTask();

    /**
     * Initializes the task for a given number of time steps
     * @param num_time_steps
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle);

    /**
     * Executes the task for the given policy parameters, and returns the costs per timestep
     * @param parameters [num_dimensions] num_parameters - policy parameters to execute
     * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
     * @return
     */
    bool execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, double& terminal_cost, const int iteration_number = 0);

    /**
     * Get the Policy object of this Task
     * @param policy
     * @return
     */
    bool getPolicy(boost::shared_ptr<policy_library::Policy>& policy);

    /**
     * Sets the Policy object of this Task
     * @param policy
     * @return
     */
    bool setPolicy(const boost::shared_ptr<policy_library::Policy> policy);

    /**
     * Gets the weight of the control cost
     * @param control_cost_weight
     * @return
     */
    bool getControlCostWeight(double& control_cost_weight);

private:
    ros::NodeHandle node_handle_;
    int num_time_steps_;
    boost::shared_ptr<policy_library::CovariantTrajectoryPolicy> policy_;

    int num_dimensions_;
    Eigen::VectorXd start_;
    Eigen::VectorXd goal_;
    Eigen::VectorXd waypoint_;
    double waypoint_time_;
    double movement_time_;

    double waypoint_cost_weight_;
    double control_cost_weight_;

    int evaluation_count_;

    void readParameters();
};

}

#endif /* COVARIANT_TRAJECTORY_WAYPOINT_TASK_H_ */
