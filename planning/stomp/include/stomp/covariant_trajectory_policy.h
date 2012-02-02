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

#ifndef STOMP_COVARIANT_TRAJECTORY_POLICY_H_
#define STOMP_COVARIANT_TRAJECTORY_POLICY_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <stomp/stomp_utils.h>

namespace stomp
{

class CovariantTrajectoryPolicy
{
public:
    CovariantTrajectoryPolicy();
    virtual ~CovariantTrajectoryPolicy();


    // Functions that are specific to CovariantTrajectoryPolicy:
    bool initialize(ros::NodeHandle& node_handle);
    bool initialize(ros::NodeHandle& node_handle,
                                               const int num_time_steps,
                                               const int num_dimensions,
                                               const double movement_duration,
                                               const double cost_ridge_factor,
                                               const std::vector<double>& derivative_costs);
    bool setToMinControlCost(Eigen::VectorXd& start, Eigen::VectorXd& goal);
    bool getParametersAll(std::vector<Eigen::VectorXd>& parameters);
    void setFileNameBase(const std::string& file_name_base);

    // Functions inherited from Policy:

    /**
     * Sets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, false on failure
     */
    bool setNumTimeSteps(const int num_time_steps);

    /**
     * Gets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, fase on failure
     */
    bool getNumTimeSteps(int& num_time_steps);

    /**
     * Gets the number of dimensions
     * @param num_dimensions (output) number of dimensions
     * @return true on success, false on failure
     */
    bool getNumDimensions(int& num_dimensions);

    /**
     * Gets the number of policy parameters per dimension
     *
     * @param num_params (output) vector of number of parameters per dimension
     * @return true on success, false on failure
     */
    bool getNumParameters(std::vector<int>& num_params);

    /**
     * Gets the basis functions that multiply the policy parameters in the dynamical system
     * @param basis_function_matrix_array (output) Array of "num_time_steps x num_parameters" matrices, per dimension
     * @return true on success, false on failure
     */
    bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions);

    /**
     * Gets the positive semi-definite matrix of the quadratic control cost
     * The weight of this control cost is provided by the task
     *
     * @param control_cost_matrix (output) Array of square, positive semi-definite matrix: num_params x num_params
     * @return true on success, false on failure
     */
    bool getControlCosts(std::vector<Eigen::MatrixXd>& control_costs);

    /**
     * Update the policy parameters based on the updates per timestep
     * @param updates (input) parameter updates per time-step, num_time_steps x num_parameters
     * @return true on success, false on failure
     */
    bool updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights);

    /**
     * Get the policy parameters per dimension
     * @param parameters (output) array of parameter vectors
     * @return true on success, false on failure
     */
    bool getParameters(std::vector<Eigen::VectorXd>& parameters);

    /**
     * Set the policy parameters per dimension
     * @param parameters (input) array of parameter vectors
     * @return true on success, false on failure
     */
    bool setParameters(const std::vector<Eigen::VectorXd>& parameters);

    /**
     * Compute the control costs over time, given the control cost matrix per dimension and parameters over time
     * @param control_cost_matrices (input) [num_dimensions] num_parameters x num_parameters: Quadratic control cost matrix (R)
     * @param parameters (input) [num_dimensions][num_time_steps] num_parameters: Parameters over time (can also be theta + projected noise)
     * @param weight (input) constant multiplier for the control costs
     * @param control_costs (output) [num_dimensions] num_time_steps: Control costs over time
     * @return
     */
    bool computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                     const double weight, std::vector<Eigen::VectorXd>& control_costs);

    bool computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<Eigen::VectorXd>& parameters,
                             const std::vector<Eigen::VectorXd>& noise, const double weight, std::vector<Eigen::VectorXd>& control_costs);

    // Functions inherited from LibraryItem:

    std::string getInfoString();
    std::string getClassName();
    bool readFromDisc(const std::string directory_name, const int item_id, const int trial_id = 0);
    bool writeToDisc(const int trial_id = 0);
    bool readFromDisc(const std::string abs_file_name);
    bool writeToDisc(const std::string abs_file_name);
    std::string getFileName(const int trial_id);


private:

    ros::NodeHandle node_handle_;

    std::string file_name_base_;

    int num_time_steps_;
    int num_vars_free_;
    int num_vars_all_;
    int free_vars_start_index_;
    int free_vars_end_index_;
    int num_dimensions_;
    double movement_duration_;
    double movement_dt_;
    double cost_ridge_factor_;
    std::vector<double> derivative_costs_;

    std::vector<int> num_parameters_;
    std::vector<Eigen::MatrixXd> basis_functions_;
    std::vector<Eigen::MatrixXd> control_costs_;
    std::vector<Eigen::MatrixXd> inv_control_costs_;
    std::vector<Eigen::MatrixXd> control_costs_all_;

    std::vector<Eigen::VectorXd> linear_control_costs_;

    std::vector<Eigen::VectorXd> parameters_all_;

    std::vector<Eigen::MatrixXd> differentiation_matrices_;
    void createDifferentiationMatrices();
    bool readParameters();
    bool initializeVariables();
    bool initializeCosts();
    bool initializeBasisFunctions();

    bool computeLinearControlCosts();
    bool computeMinControlCostParameters();
};

// inline functions follow

inline bool CovariantTrajectoryPolicy::getParameters(std::vector<Eigen::VectorXd>& parameters)
{
    if (int(parameters.size()) != num_dimensions_)
    {
        parameters.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
    }
    for (int d=0; d<num_dimensions_; ++d)
    {
        parameters[d] = parameters_all_[d].segment(free_vars_start_index_, num_vars_free_);
    }
    return true;
}

inline bool CovariantTrajectoryPolicy::getParametersAll(std::vector<Eigen::VectorXd>& parameters)
{
    parameters = parameters_all_;
    return true;
}

inline bool CovariantTrajectoryPolicy::setParameters(const std::vector<Eigen::VectorXd>& parameters)
{
    ROS_ASSERT(int(parameters.size()) == num_dimensions_);
    for (int d=0; d<num_dimensions_; ++d)
    {
        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) = parameters[d];
    }
    return true;
}

inline bool CovariantTrajectoryPolicy::getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions)
{
    basis_functions = basis_functions_;
    return true;
}

inline bool CovariantTrajectoryPolicy::getControlCosts(std::vector<Eigen::MatrixXd>& control_costs)
{
    control_costs = control_costs_;
    return true;
}

inline bool CovariantTrajectoryPolicy::getNumTimeSteps(int& num_time_steps)
{
    num_time_steps = num_time_steps_;
    return true;
}

inline bool CovariantTrajectoryPolicy::getNumDimensions(int& num_dimensions)
{
    num_dimensions = num_dimensions_;
    return true;
}

inline bool CovariantTrajectoryPolicy::getNumParameters(std::vector<int>& num_params)
{
    num_params = num_parameters_;
    return true;
}

inline bool CovariantTrajectoryPolicy::setNumTimeSteps(const int num_time_steps)
{
    ROS_ASSERT_MSG(num_time_steps_ == num_time_steps, "%d != %d", num_time_steps_, num_time_steps);
    return true;
}

inline std::string CovariantTrajectoryPolicy::getInfoString()
{
    return "";
}

inline std::string CovariantTrajectoryPolicy::getClassName()
{
    return "CovariantTrajectoryPolicy";
}

inline void CovariantTrajectoryPolicy::setFileNameBase(const std::string& file_name_base)
{
    file_name_base_ = file_name_base;
}


}

#endif /* STOMP_COVARIANT_TRAJECTORY_POLICY_H_ */
