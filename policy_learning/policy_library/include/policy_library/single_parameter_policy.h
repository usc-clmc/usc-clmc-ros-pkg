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

#ifndef SINGLEPARAMETERPOLICY_H_
#define SINGLEPARAMETERPOLICY_H_

// ros includes


// local includes
#include <policy_library/policy.h>
#include <policy_msgs/SingleParameterPolicy.h>

namespace policy_library
{

class SingleParameterPolicy : public policy_library::Policy
{
public:
    SingleParameterPolicy();
    virtual ~SingleParameterPolicy();

    // functions specific to SingleParameterPolicy:
    bool initialize(int num_dimensions);

    // inherited from Policy:
    bool setNumTimeSteps(const int num_time_steps);
    bool getNumTimeSteps(int& num_time_steps);
    bool getNumDimensions(int& num_dimensions);
    bool getNumParameters(std::vector<int>& num_params);
    bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions);
    bool getControlCosts(std::vector<Eigen::MatrixXd>& control_costs);
    bool updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights);
    bool getParameters(std::vector<Eigen::VectorXd>& parameters);
    bool setParameters(const std::vector<Eigen::VectorXd>& parameters);

    std::string getClassName();
    bool readFromFile(const std::string& abs_bagfile_name);
    bool writeToFile(const std::string& abs_bagfile_name);

private:

    bool initialized_;

    int num_dimensions_;
    int num_time_steps_;
    std::vector<Eigen::VectorXd> parameters_;

    bool initFromMessage(const policy_msgs::SingleParameterPolicy& spp_msg);
    bool writeToMessage(policy_msgs::SingleParameterPolicy& spp_msg);
};

}

#endif /* SINGLEPARAMETERPOLICY_H_ */
