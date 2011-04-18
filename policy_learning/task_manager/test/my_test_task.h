/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

  \file    my_test_task.h

  \author  Peter Pastor
  \date    Jun 10, 2010

**********************************************************************/

#ifndef MY_TEST_TASK_H_
#define MY_TEST_TASK_H_

// system includes
#include <string>

// ros includes
#include <ros/ros.h>


// local includes
#include <task_manager/task.h>

namespace my_test_task
{

class TestTask : public task_manager_interface::Task
{

public:

    TestTask();
    ~TestTask();

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
     * Reset the system to a state where the next task can be executed
     * @return
     */
    bool reset();

    bool getPolicy(boost::shared_ptr<policy_library::Policy>& policy);
    bool setPolicy(const boost::shared_ptr<policy_library::Policy> policy);
    bool getControlCostWeight(double& control_cost_weight);


private:

    bool initialized_;
};

}

#endif /* MY_TEST_TASK_H_ */
