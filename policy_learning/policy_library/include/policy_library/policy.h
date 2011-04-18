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

#ifndef POLICY_H_
#define POLICY_H_

#include <vector>

#include <Eigen/Core>

namespace policy_library
{

class Policy
{
public:

    Policy(){};
    virtual ~Policy(){};

    /**
     * Sets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, false on failure
     */
    virtual bool setNumTimeSteps(const int num_time_steps) = 0;

    /**
     * Gets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, fase on failure
     */
    virtual bool getNumTimeSteps(int& num_time_steps) = 0;

    /**
     * Gets the number of dimensions
     * @param num_dimensions (output) number of dimensions
     * @return true on success, false on failure
     */
    virtual bool getNumDimensions(int& num_dimensions) = 0;

    /**
     * Gets the number of policy parameters per dimension
     *
     * @param num_params (output) vector of number of parameters per dimension
     * @return true on success, false on failure
     */
    virtual bool getNumParameters(std::vector<int>& num_params) = 0;

    /**
     * Gets the basis functions that multiply the policy parameters in the dynamical system
     * @param basis_function_matrix_array (output) Array of "num_time_steps x num_parameters" matrices, per dimension
     * @return true on success, false on failure
     */
    virtual bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions) = 0;

    /**
     * Gets the positive semi-definite matrix of the quadratic control cost
     * The weight of this control cost is provided by the task
     *
     * @param control_cost_matrix (output) Array of square, positive semi-definite matrix: num_params x num_params
     * @return true on success, false on failure
     */
    virtual bool getControlCosts(std::vector<Eigen::MatrixXd>& control_costs) = 0;

    /**
     * Update the policy parameters based on the updates per timestep
     * @param updates (input) parameter updates per time-step, num_time_steps x num_parameters
     * @return true on success, false on failure
     */
    virtual bool updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights) = 0;

    /**
     * Get the policy parameters per dimension
     * @param parameters (output) array of parameter vectors
     * @return true on success, false on failure
     */
    virtual bool getParameters(std::vector<Eigen::VectorXd>& parameters) = 0;

    /**
     * Set the policy parameters per dimension
     * @param parameters (input) array of parameter vectors
     * @return true on success, false on failure
     */
    virtual bool setParameters(const std::vector<Eigen::VectorXd>& parameters) = 0;

    /**
     * Compute the control costs over time, given the control cost matrix per dimension and parameters over time
     * @param control_cost_matrices (input) [num_dimensions] num_parameters x num_parameters: Quadratic control cost matrix (R)
     * @param parameters (input) [num_dimensions][num_time_steps] num_parameters: Parameters over time (can also be theta + projected noise)
     * @param weight (input) constant multiplier for the control costs
     * @param control_costs (output) [num_dimensions] num_time_steps: Control costs over time
     * @return
     */
    virtual bool computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                     const double weight, std::vector<Eigen::VectorXd>& control_costs);

    /**
     * Read the policy from file
     * @param abs_file_name
     * @return
     */
    virtual bool readFromFile(const std::string& abs_file_name) = 0;

    /**
     * Write the policy to file
     * @param abs_file_name
     * @return
     */
    virtual bool writeToFile(const std::string& abs_file_name) = 0;

    /**
     * Get the name of the class
     * @return
     */
    virtual std::string getClassName() = 0;

};

}

#endif /* POLICY_H_ */
