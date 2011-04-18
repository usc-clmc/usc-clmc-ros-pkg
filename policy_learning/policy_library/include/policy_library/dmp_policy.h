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

 \file    dmp_policy.h

 \author  Peter Pastor
 \date    Jun 10, 2010

 **********************************************************************/

#ifndef DMP_POLICY_H_
#define DMP_POLICY_H_

// system includes

// ros includes
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include <dmp_lib/dynamic_movement_primitive.h>

// local includes
#include <policy_library/policy.h>

namespace policy_library
{

class DMPPolicy : public Policy
{

public:

  /*! Constructor
   */
  DMPPolicy() :
    initialized_(false), num_time_steps_(-1) {};
  /*! Destructor
   */
  virtual ~DMPPolicy() {};

  /*!
   * Initializes the DMPPolicy object with a pointer to a already initialized dmp
   * @param dmp
   * @return True on success, otherwise False
   */
  bool initialize(dmp_lib::DMPPtr dmp);

  /**
   * Retrieves the underlying DMP from the policy
   * @param dmp
   * @return True on success, otherwise False
   */
  bool getDMP(dmp_lib::DMPPtr& dmp);

  /**
   * Sets the number of time steps used in reinforcement learning
   * @param num_time_steps
   * @return True on success, otherwise False
   */
  bool setNumTimeSteps(const int num_time_steps);

  /**
   * Gets the number of time steps used in reinforcement learning
   * @param num_time_steps
   * @return True on success, otherwise False
   */
  bool getNumTimeSteps(int& num_time_steps);

  /**
   * Gets the number of dimensions
   * @param num_dimensions (output) number of dimensions
   * @return True on success, otherwise False
   */
  bool getNumDimensions(int& num_dimensions);

  /*!
   * Gets the number of policy parameters per dimension
   *
   * @param num_params (output) vector of number of parameters per dimension
   * @return True on success, otherwise False
   */
  bool getNumParameters(std::vector<int>& num_params);

  /*!
   * Gets the basis functions that multiply the policy parameters in the dynamical system
   * @param basis_function_matrix_array (output) Array of "num_time_steps x num_parameters" matrices, per dimension
   * @return True on success, otherwise False
   */
  bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions);

  /*!
   * Compute the control costs over time, given the control cost matrix per dimension and parameters over time
   * @param control_cost_matrices (input) [num_dimensions] num_parameters x num_parameters: Quadratic control cost matrix (R)
   * @param parameters (input) [num_dimensions][num_time_steps] num_parameters: Parameters over time (can also be theta + projected noise)
   * @param weight (input) constant multiplier for the control costs
   * @param control_costs (output) [num_dimensions] num_time_steps: Control costs over time
   * @return True on success, otherwise False
   */
  bool computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices,
                           const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                           const double weight,
                           std::vector<Eigen::VectorXd>& control_costs);

  /*!
   * Gets the positive semi-definite matrix of the quadratic control cost
   * The weight of this control cost is provided by the task
   *
   * @param control_cost_matrix (output) Array of square, positive semi-definite matrix: num_params x num_params
   * @return True on success, otherwise False
   */
  bool getControlCosts(std::vector<Eigen::MatrixXd>& control_costs);

  /*!
   * Update the policy parameters based on the updates per timestep
   * @param updates (input) parameter updates per time-step, num_time_steps x num_parameters
   * @return True on success, otherwise False
   */
  bool updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights);

  /*!
   * Get the policy parameters per dimension
   * @param parameters (output) array of parameter vectors
   * @return True on success, otherwise False
   */
  bool getParameters(std::vector<Eigen::VectorXd>& parameters);

  /*!
   * Set the policy parameters per dimension
   * @param parameters (input) array of parameter vectors
   * @return True on success, otherwise False
   */
  bool setParameters(const std::vector<Eigen::VectorXd>& parameters);

  /**
   * Read the policy from file
   * @param abs_file_name
   * @return True on success, otherwise False
   */
  bool readFromFile(const std::string& abs_file_name);

  /**
   * Write the policy to file
   * @param abs_file_name
   * @return True on success, otherwise False
   */
  bool writeToFile(const std::string& abs_file_name);

  /**
   * Get the name of the class
   * @return
   */
  std::string getClassName();

private:

  /*!
   */
  bool initialized_;
  int num_time_steps_;

  /*!
   */
  dmp_lib::DMPPtr dmp_;

};

}

#endif /* DMP_POLICY_H_ */
