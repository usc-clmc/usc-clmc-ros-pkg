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

 \file    policy_improvement.cpp

 \author  Ludovic Righetti, Peter Pastor, Mrinal Kalakrishnan
 \date    May 26, 2010

 **********************************************************************/

// system includes
#include <time.h>
#include <cfloat>
#include <ros/assert.h>

#include <Eigen/LU>
#include <Eigen/Core>

// local includes
#include <ros/ros.h>
#include <policy_improvement/policy_improvement.h>
#include <usc_utilities/assert.h>
#include <algorithm>

using namespace Eigen;
using namespace policy_library;

namespace pi2
{

PolicyImprovement::PolicyImprovement():
    initialized_(false)
{
}

PolicyImprovement::~PolicyImprovement()
{
}

bool PolicyImprovement::initialize(const int num_rollouts, const int num_time_steps, const int num_reused_rollouts,
                                   const int num_extra_rollouts, boost::shared_ptr<Policy> policy,
                                   bool use_cumulative_costs, bool reevaluate_reused_rollouts)
{
    num_time_steps_ = num_time_steps;
    use_cumulative_costs_ = use_cumulative_costs;
    reevaluate_reused_rollouts_ = reevaluate_reused_rollouts;
    policy_ = policy;

    ROS_VERIFY(policy_->setNumTimeSteps(num_time_steps_));
    ROS_VERIFY(policy_->getControlCosts(control_costs_));
    ROS_VERIFY(policy_->getNumDimensions(num_dimensions_));
    ROS_VERIFY(policy_->getNumParameters(num_parameters_));
    ROS_VERIFY(policy_->getBasisFunctions(basis_functions_));
    ROS_VERIFY(policy_->getParameters(parameters_));

    // invert the control costs, initialize noise generators:
    inv_control_costs_.clear();
    noise_generators_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
        inv_control_costs_.push_back(control_costs_[d].fullPivLu().inverse());
        MultivariateGaussian mvg(VectorXd::Zero(num_parameters_[d]), inv_control_costs_[d]);
        noise_generators_.push_back(mvg);
    }

    ROS_VERIFY(setNumRollouts(num_rollouts, num_reused_rollouts, num_extra_rollouts));
    ROS_VERIFY(preAllocateTempVariables());
    ROS_VERIFY(preComputeProjectionMatrices());

    return (initialized_ = true);
}

bool PolicyImprovement::setNumRollouts(const int num_rollouts, const int num_reused_rollouts, const int num_extra_rollouts)
{
    num_rollouts_ = num_rollouts;
    num_rollouts_reused_ = num_reused_rollouts;
    num_rollouts_extra_ = num_extra_rollouts;
    num_rollouts_gen_ = 0;
    if (num_rollouts_reused_ >= num_rollouts)
    {
        ROS_ERROR("Number of reused rollouts must be strictly less than number of rollouts.");
        return false;
    }

    // preallocate memory for a single rollout:
    Rollout rollout;

    rollout.parameters_.clear();
    rollout.noise_.clear();
    rollout.noise_projected_.clear();
    rollout.parameters_noise_projected_.clear();
    rollout.control_costs_.clear();
    rollout.total_costs_.clear();
    rollout.cumulative_costs_.clear();
    rollout.probabilities_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
        rollout.parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
        rollout.noise_.push_back(VectorXd::Zero(num_parameters_[d]));
        std::vector<VectorXd> tmp_projected_noise;
        for (int t=0; t<num_time_steps_; ++t)
        {
            tmp_projected_noise.push_back(VectorXd::Zero(num_parameters_[d]));
        }
        rollout.noise_projected_.push_back(tmp_projected_noise);
        rollout.parameters_noise_projected_.push_back(tmp_projected_noise);
        rollout.control_costs_.push_back(VectorXd::Zero(num_time_steps_));
        rollout.total_costs_.push_back(VectorXd::Zero(num_time_steps_));
        rollout.cumulative_costs_.push_back(VectorXd::Zero(num_time_steps_));
        rollout.probabilities_.push_back(VectorXd::Zero(num_time_steps_));
    }
    rollout.state_costs_ = VectorXd::Zero(num_time_steps_);

    // duplicate this rollout:
    for (int r=0; r<num_rollouts; ++r)
        rollouts_.push_back(rollout);

    for (int r=0; r<num_reused_rollouts; ++r)
        reused_rollouts_.push_back(rollout);

    for (int r=0; r<num_extra_rollouts; ++r)
        extra_rollouts_.push_back(rollout);

    rollouts_reused_ = false;
    rollouts_reused_next_ = false;
    extra_rollouts_added_ = false;
    rollout_cost_sorter_.reserve(num_rollouts_);

    return true;
}

double Rollout::getCost()
{
    double cost = state_costs_.sum();
    int num_dim = control_costs_.size();
    for (int d=0; d<num_dim; ++d)
        cost += control_costs_[d].sum();
    return cost + terminal_cost_;
}

bool PolicyImprovement::generateRollouts(const std::vector<double>& noise_stddev)
{
    ROS_ASSERT(initialized_);
    ROS_ASSERT(static_cast<int>(noise_stddev.size()) == num_dimensions_);

    // save the latest policy parameters:
    ROS_VERIFY(copyParametersFromPolicy());

    // we assume here that rollout_parameters_ and rollout_noise_ have already been allocated
    num_rollouts_gen_ = num_rollouts_ - num_rollouts_reused_;
    if (!rollouts_reused_next_)
    {
        num_rollouts_gen_ = num_rollouts_;
        if (num_rollouts_reused_ > 0)
        {
            rollouts_reused_next_ = true;
        }
    }
    else
    {
        // figure out which rollouts to reuse
        rollout_cost_sorter_.clear();
        for (int r=0; r<num_rollouts_; ++r)
        {
            double cost = rollouts_[r].getCost();
            rollout_cost_sorter_.push_back(std::make_pair(cost,r));
        }
        if (extra_rollouts_added_)
        {
            for (int r=0; r<num_rollouts_extra_; ++r)
            {
                double cost = extra_rollouts_[r].getCost();
                rollout_cost_sorter_.push_back(std::make_pair(cost,-r-1));
                // index is -ve if rollout is taken from extra_rollouts
            }
            extra_rollouts_added_ = false;
        }
        std::sort(rollout_cost_sorter_.begin(), rollout_cost_sorter_.end());

        // use the best ones: (copy them into reused_rollouts)
        for (int r=0; r<num_rollouts_reused_; ++r)
        {
            double reuse_index = rollout_cost_sorter_[r].second;
            //double reuse_cost = rollout_cost_sorter_[r].first;

            //ROS_INFO("Reuse %d, cost = %lf", r, reuse_cost);

            if (reuse_index >=0)
                reused_rollouts_[r] = rollouts_[reuse_index];
            else
            {
                //ROS_INFO("Reused noise-less rollout of cost %f", reuse_cost);
                reused_rollouts_[r] = extra_rollouts_[-reuse_index-1];
            }
        }
        // copy them back from reused_rollouts_ into rollouts_
        for (int r=0; r<num_rollouts_reused_; ++r)
        {
            rollouts_[num_rollouts_gen_+r] = reused_rollouts_[r];

            // update the noise based on the new parameters:
            for (int d=0; d<num_dimensions_; ++d)
            {
                rollouts_[num_rollouts_gen_+r].noise_[d] = rollouts_[num_rollouts_gen_+r].parameters_[d] - parameters_[d];
            }
        }
        rollouts_reused_ = true;
    }

    // generate new rollouts
    for (int d=0; d<num_dimensions_; ++d)
    {
        for (int r=0; r<num_rollouts_gen_; ++r)
        {
            noise_generators_[d].sample(tmp_noise_[d]);
            rollouts_[r].noise_[d] = noise_stddev[d]*tmp_noise_[d];
            rollouts_[r].parameters_[d] = parameters_[d] + rollouts_[r].noise_[d];
        }
    }

    if (rollouts_reused_ && reevaluate_reused_rollouts_)
    {
      num_rollouts_gen_ = num_rollouts_;
    }

    return true;
}

bool PolicyImprovement::getRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, const std::vector<double>& noise_variance)
{
    if (!generateRollouts(noise_variance))
    {
        ROS_ERROR("Failed to generate rollouts.");
        return false;
    }

    rollouts.clear();
    for (int r=0; r<num_rollouts_gen_; ++r)
    {
        rollouts.push_back(rollouts_[r].parameters_);
    }

    computeProjectedNoise();

    return true;
}

bool PolicyImprovement::setRolloutCosts(const Eigen::MatrixXd& costs, const Eigen::VectorXd& terminal_costs, const double control_cost_weight, std::vector<double>& rollout_costs_total)
{
    ROS_ASSERT(initialized_);

    control_cost_weight_ = control_cost_weight;
    computeRolloutControlCosts();

    for (int r=0; r<num_rollouts_gen_; ++r)
    {
        rollouts_[r].state_costs_ = costs.row(r).transpose();
        rollouts_[r].terminal_cost_ = terminal_costs(r);
    }

    // set the total costs
    rollout_costs_total.resize(num_rollouts_);
    for (int r=0; r<num_rollouts_; ++r)
    {
        rollout_costs_total[r] = rollouts_[r].getCost();
    }
    return true;
}

bool PolicyImprovement::computeProjectedNoise()
{
    for (int r=0; r<num_rollouts_; ++r)
    {
        computeProjectedNoise(rollouts_[r]);
    }
    return true;
}

bool PolicyImprovement::computeRolloutControlCosts()
{
    for (int r=0; r<num_rollouts_; ++r)
    {
        computeRolloutControlCosts(rollouts_[r]);
    }
    return true;
}

bool PolicyImprovement::computeRolloutCumulativeCosts()
{
    // compute cumulative costs at each timestep
    for (int r=0; r<num_rollouts_; ++r)
    {
        for (int d=0; d<num_dimensions_; ++d)
        {
            rollouts_[r].total_costs_[d] = rollouts_[r].state_costs_ + rollouts_[r].control_costs_[d];
            rollouts_[r].cumulative_costs_[d] = rollouts_[r].total_costs_[d];
            if (use_cumulative_costs_)
            {
              // add the terminal cost to the last state cost, and perform backwards cumulation
              rollouts_[r].cumulative_costs_[d](num_time_steps_-1) += rollouts_[r].terminal_cost_;
              for (int t=num_time_steps_-2; t>=0; --t)
              {
                  rollouts_[r].cumulative_costs_[d](t) += rollouts_[r].cumulative_costs_[d](t+1);
              }
            }
            else
            {
              // just add the terminal cost to all state costs
              for (int t=num_time_steps_-1; t>=0; --t)
              {
                rollouts_[r].cumulative_costs_[d](t) += rollouts_[r].terminal_cost_;
              }
            }
        }
    }
    return true;
}

bool PolicyImprovement::computeRolloutProbabilities()
{
    for (int d=0; d<num_dimensions_; ++d)
    {

        for (int t=0; t<num_time_steps_; t++)
        {

            // find min and max cost over all rollouts:
            double min_cost = rollouts_[0].cumulative_costs_[d](t);
            double max_cost = min_cost;
            for (int r=1; r<num_rollouts_; ++r)
            {
                double c = rollouts_[r].cumulative_costs_[d](t);
                if (c < min_cost)
                    min_cost = c;
                if (c > max_cost)
                    max_cost = c;
            }

            double denom = max_cost - min_cost;

            //time_step_weights_[d][t] = denom;
            time_step_weights_[d][t] = 1.0;

            // prevent divide by zero:
            if (denom < 1e-8)
                denom = 1e-8;

            double p_sum = 0.0;
            for (int r=0; r<num_rollouts_; ++r)
            {
                // the -10.0 here is taken from the paper:
                rollouts_[r].probabilities_[d](t) = exp(-10.0*(rollouts_[r].cumulative_costs_[d](t) - min_cost)/denom);
                p_sum += rollouts_[r].probabilities_[d](t);
            }
            for (int r=0; r<num_rollouts_; ++r)
            {
                rollouts_[r].probabilities_[d](t) /= p_sum;
            }

        }

    }
    return true;
}

bool PolicyImprovement::computeParameterUpdates()
{
    for (int d=0; d<num_dimensions_; ++d)
    {
        parameter_updates_[d] = MatrixXd::Zero(num_time_steps_, num_parameters_[d]);
        for (int t=0; t<num_time_steps_; ++t)
        {
            for (int r=0; r<num_rollouts_; ++r)
            {
                parameter_updates_[d].row(t).transpose() += rollouts_[r].noise_projected_[d][t] * rollouts_[r].probabilities_[d](t);
            }
        }
    }
    return true;
}

bool PolicyImprovement::improvePolicy(std::vector<Eigen::MatrixXd>& parameter_updates)
{
    ROS_ASSERT(initialized_);

    computeRolloutCumulativeCosts();
    computeRolloutProbabilities();
    computeParameterUpdates();
    parameter_updates = parameter_updates_;

    return true;
}

bool PolicyImprovement::preAllocateTempVariables()
{
    tmp_noise_.clear();
    tmp_parameters_.clear();
    parameter_updates_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
        tmp_noise_.push_back(VectorXd::Zero(num_parameters_[d]));
        tmp_parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
        parameter_updates_.push_back(MatrixXd::Zero(num_time_steps_, num_parameters_[d]));
        time_step_weights_.push_back(VectorXd::Zero(num_time_steps_));
    }
    tmp_sum_rollout_probabilities_ = VectorXd::Zero(num_time_steps_);

    return true;
}

bool PolicyImprovement::preComputeProjectionMatrices()
{
    projection_matrices_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
        std::vector<MatrixXd> projection_matrices_for_dim;

        VectorXd basis_function(num_parameters_[d]);
        VectorXd inv_r_times_g(num_parameters_[d]);
        for (int t=0; t<num_time_steps_; ++t)
        {
            basis_function = basis_functions_[d].row(t).transpose();
            inv_r_times_g = inv_control_costs_[d] * basis_function;
            double g_transpose_r_g = basis_function.dot(inv_r_times_g);

            if (g_transpose_r_g < 1e-6)
            {
                ROS_WARN("Denominator (g_transpose_r_g) is close to 0: %f", g_transpose_r_g);
            }

            // outer product
            projection_matrices_for_dim.push_back((inv_r_times_g / g_transpose_r_g) * basis_function.transpose());
        }
        projection_matrices_.push_back(projection_matrices_for_dim);

    }
    return true;
}

bool PolicyImprovement::addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rollout_costs,
                                         std::vector<double>& rollout_terminal_costs)
{
    ROS_ASSERT(int(rollouts.size()) == num_rollouts_extra_);

    // update our parameter values, so that the computed noise is correct:
    ROS_VERIFY(copyParametersFromPolicy());

    for (int r=0; r<num_rollouts_extra_; ++r)
    {
        extra_rollouts_[r].parameters_ = rollouts[r];
        extra_rollouts_[r].state_costs_ = rollout_costs[r];
        extra_rollouts_[r].terminal_cost_ = rollout_terminal_costs[r];
        computeNoise(extra_rollouts_[r]);
        computeProjectedNoise(extra_rollouts_[r]);
        computeRolloutControlCosts(extra_rollouts_[r]);
        //ROS_INFO("Extra rollout cost = %f", extra_rollouts_[r].getCost());
    }

    extra_rollouts_added_ = true;
    return true;
}

bool PolicyImprovement::computeNoise(Rollout& rollout)
{
    for (int d=0; d<num_dimensions_; ++d)
    {
        rollout.noise_[d] =  rollout.parameters_[d] - parameters_[d];
    }
    return true;
}

bool PolicyImprovement::computeProjectedNoise(Rollout& rollout)
{
    for (int d=0; d<num_dimensions_; ++d)
    {
        for (int t=0; t<num_time_steps_; ++t)
        {
            rollout.noise_projected_[d][t] = projection_matrices_[d][t] * rollout.noise_[d];
            rollout.parameters_noise_projected_[d][t] = rollout.parameters_[d] + rollout.noise_projected_[d][t];
        }
    }
    return true;
}

bool PolicyImprovement::computeRolloutControlCosts(Rollout& rollout)
{
    policy_->computeControlCosts(control_costs_, rollout.parameters_noise_projected_,
                                 0.5*control_cost_weight_, rollout.control_costs_);
    return true;
}

bool PolicyImprovement::copyParametersFromPolicy()
{
    if (!policy_->getParameters(parameters_))
    {
        ROS_ERROR("Failed to get policy parameters.");
        return false;
    }
    return true;
}

bool PolicyImprovement::getTimeStepWeights(std::vector<Eigen::VectorXd>& time_step_weights)
{
  time_step_weights = time_step_weights_;
  return true;
}

};
