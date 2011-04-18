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

  \file    test_task.cpp

  \author  Peter Pastor
  \date    Jun 10, 2010

**********************************************************************/


// system includes

// ros includes
#include <pluginlib/class_list_macros.h>
#include <task_manager/task_manager.h>

// local includes
#include "my_test_task.h"

PLUGINLIB_DECLARE_CLASS(task_manager, TestTask, my_test_task::TestTask, task_manager_interface::Task)

namespace my_test_task
{

TestTask::TestTask()
{

}
TestTask::~TestTask()
{

}

bool TestTask::initialize(ros::NodeHandle& node_handle)
{
    ROS_INFO("initialize...");

    initialized_ = true;
    return initialized_;
}

bool TestTask::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, double& terminal_cost, const int iteration_number)
{
    ROS_INFO("execute...");

    return true;
}

bool TestTask::reset()
{
    return true;
}

bool TestTask::getPolicy(boost::shared_ptr<policy_library::Policy>& policy)
{
    return true;
}

bool TestTask::setPolicy(const boost::shared_ptr<policy_library::Policy> policy)
{
    return true;
}

bool TestTask::getControlCostWeight(double& control_cost_weight)
{
    return true;
}


}

