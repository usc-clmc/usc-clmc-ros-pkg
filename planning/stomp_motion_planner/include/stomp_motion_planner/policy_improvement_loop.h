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

#ifndef POLICY_IMPROVEMENT_LOOP_H_
#define POLICY_IMPROVEMENT_LOOP_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <boost/shared_ptr.hpp>

#include <stomp_motion_planner/policy.h>

#include <stomp_motion_planner/task.h>
#include <stomp_motion_planner/policy_improvement.h>
//#include <policy_improvement_loop/PolicyImprovementStatistics.h>

namespace stomp_motion_planner
{

class PolicyImprovementLoop
{
public:
    PolicyImprovementLoop();
    virtual ~PolicyImprovementLoop();

    //bool initializeAndRunTaskByName(ros::NodeHandle& node_handle, std::string& task_name);

    bool initialize(ros::NodeHandle& node_handle, boost::shared_ptr<Task> task);
    bool runSingleIteration(int iteration_number);
    void clearReusedRollouts();

private:

    bool initialized_;
    ros::NodeHandle node_handle_;

    int num_rollouts_;
    int num_reused_rollouts_;
    int num_time_steps_;
    int num_dimensions_;

    bool write_to_file_;
    bool use_cumulative_costs_;

    boost::shared_ptr<Task> task_;
    boost::shared_ptr<Policy> policy_;

    PolicyImprovement policy_improvement_;

    std::vector<std::vector<Eigen::VectorXd> > rollouts_; /**< [num_rollouts][num_dimensions] num_parameters */
    std::vector<Eigen::MatrixXd> parameter_updates_;
    std::vector<Eigen::VectorXd> parameters_;
    std::vector<Eigen::VectorXd> time_step_weights_;
    Eigen::MatrixXd rollout_costs_;
    std::vector<double> noise_stddev_;
    std::vector<double> noise_decay_;
    double control_cost_weight_;

    // temporary variables
    Eigen::VectorXd tmp_rollout_cost_;

    bool readParameters();

    int policy_iteration_counter_;
    bool readPolicy(const int iteration_number);
    bool writePolicy(const int iteration_number, bool is_rollout = false, int rollout_id = 0);

    //bool writePolicyImprovementStatistics(const policy_improvement_loop::PolicyImprovementStatistics& stats_msg);

};

}

#endif /* POLICY_IMPROVEMENT_LOOP_H_ */
