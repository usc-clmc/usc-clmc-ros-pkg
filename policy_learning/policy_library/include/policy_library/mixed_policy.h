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

#ifndef MIXED_POLICY_H_
#define MIXED_POLICY_H_

#include <policy_library/policy.h>

namespace policy_library
{

/**
 * Combines two or more different kinds of policies into a single policy that PolicyImprovement can optimize
 */
class MixedPolicy: public Policy
{
public:
    MixedPolicy();
    virtual ~MixedPolicy();

    // functions specific to MixedPolicy:
    bool initialize(std::vector<boost::shared_ptr<Policy> >& policies);

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

    /**
     * Read the policy from file
     * @param abs_file_name
     * @return
     */
    bool readFromFile(const std::string& abs_file_name);

    /**
     * Write the policy to file
     * @param abs_file_name
     * @return
     */
    bool writeToFile(const std::string& abs_file_name);

    std::string getClassName();

private:

    bool initialized_;
    std::string library_directory_name_;
    int id_;
    int trial_id_;

    int num_dimensions_;
    std::vector<int> num_dimensions_per_policy_;
    int num_time_steps_;
    int num_policies_;
    std::vector<boost::shared_ptr<Policy> > policies_;

};

}

#endif /* MIXED_POLICY_H_ */
