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

#include <gtest/gtest.h>
#include <policy_library/covariant_trajectory_policy.h>
#include <Eigen/Core>

using namespace policy_library;
using namespace Eigen;

TEST(CovariantTrajectoryPolicyTest, testMinControlCost)
{
    ros::NodeHandle node_handle;
    /*CovariantTrajectoryPolicy ctp;
    ctp.initialize(node_handle);

    VectorXd start = VectorXd::Zero(1);
    VectorXd goal = VectorXd::Zero(1);

    start(0) = 2.0;
    goal(0) = 1.0;

    ctp.setToMinControlCost(start, goal);

    std::vector<VectorXd> params;
    ctp.getParametersAll(params);
    ROS_INFO_STREAM(params[0]);*/
}
