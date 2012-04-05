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
#include <boost/shared_ptr.hpp>
#include <policy_library/policy.h>
#include <policy_improvement/policy_improvement.h>
#include <iostream>

using namespace Eigen;
using namespace pi2;
using namespace std;
using namespace policy_library;

static double getDummyPolicyCost(double parameter)
{
    return (parameter - 1.0)*(parameter - 1.0);
}

class DummyPolicy: public Policy
{
public:
    DummyPolicy()
    {
        parameter_ = 0;
    }

    ~DummyPolicy()
    {
    }

    // implementation of abstract functions from Policy

    bool setNumTimeSteps(const int num_time_steps)
    {
        return true;
    }

    bool getNumTimeSteps(int& num_time_steps)
    {
        num_time_steps = 1;
        return true;
    }

    bool getNumDimensions(int& num_dimensions)
    {
        num_dimensions = 1;
        return true;
    }

    bool getNumParameters(std::vector<int>& num_params)
    {
        num_params.clear();
        num_params.push_back(1);
        return true;
    }

    bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions)
    {
        basis_functions.clear();
        basis_functions.push_back(MatrixXd::Identity(1,1));
        return true;
    }

    bool getControlCosts(std::vector<Eigen::MatrixXd>& control_costs)
    {
        control_costs.clear();
        control_costs.push_back(MatrixXd::Identity(1,1));
        return true;
    }

    bool updateParameters(const std::vector<Eigen::MatrixXd>& updates)
    {
        parameter_ += updates[0](0,0);
        return true;
    }

    bool getParameters(std::vector<Eigen::VectorXd>& parameters)
    {
        parameters.clear();
        VectorXd p = VectorXd(1);
        p(0) = parameter_;
        parameters.push_back(p);
        return true;
    }

    bool setParameters(const std::vector<Eigen::VectorXd>& parameters)
    {
        parameter_ = parameters[0](0);
        return true;
    }

    // implementation of some extra functions:

    double getCost()
    {
        return getDummyPolicyCost(parameter_);
    }

    bool readFromFile(const std::string& abs_file_name)
    {
        return true;
    }

    bool writeToFile(const std::string& abs_file_name)
    {
        return true;
    }

    std::string getClassName()
    {
        return "DummyPolicy";
    }

private:
    double parameter_; // the dummy policy just has a single parameter in one dimension
};

double optimizeDummyPolicy(int num_rollouts, int num_reused_rollouts, int num_extra_rollouts, int num_iterations)
{
    int num_timesteps = 1;
    double noise = 0.5;
    double noise_decay = 0.999;
    double control_cost_weight = 0.00000001;

    DummyPolicy *dummy_policy = new DummyPolicy();
    boost::shared_ptr<Policy> policy(dummy_policy);
    PolicyImprovement policy_improvement;

    policy_improvement.initialize(num_rollouts, num_timesteps, num_reused_rollouts, num_extra_rollouts, policy);

    std::vector<std::vector<Eigen::VectorXd> > rollouts;
    std::vector<Eigen::MatrixXd> parameter_updates;
    std::vector<Eigen::VectorXd> parameters;
    Eigen::MatrixXd rollout_costs = MatrixXd::Zero(num_rollouts, 1);
    std::vector<double> noise_vector;
    noise_vector.push_back(noise);

    // the PI^2 loop
    for (int i=0; i<num_iterations; ++i)
    {
        policy->getParameters(parameters);
        double noiseless_cost = dummy_policy->getCost();
        cout << "Policy: " << parameters[0](0) << " Noise-less cost at iteration: " << noiseless_cost << endl;

        if (num_extra_rollouts>0)
        {
            // send the noise-less policy as the extra rollout:
            std::vector<std::vector<Eigen::VectorXd> > extra_rollout;
            std::vector<Eigen::VectorXd> extra_rollout_cost;
            extra_rollout.resize(1);
            extra_rollout[0] = parameters;
            extra_rollout_cost.resize(1,Eigen::VectorXd::Zero(1));
            extra_rollout_cost[0](0) = noiseless_cost;
            policy_improvement.addExtraRollouts(extra_rollout, extra_rollout_cost);
        }

        policy_improvement.getRollouts(rollouts, noise_vector);
        for (int r=0; r<int(rollouts.size()); ++r)
        {
            rollout_costs(r,0) = getDummyPolicyCost(rollouts[r][0](0));
        }
        std::vector<double> all_costs;
        policy_improvement.setRolloutCosts(rollout_costs, control_cost_weight, all_costs);
        policy_improvement.improvePolicy(parameter_updates);
        policy->updateParameters(parameter_updates);
        noise_vector[0] *= noise_decay;
    }
    return dummy_policy->getCost();
}


TEST(TestDummyPolicy, dummyPolicyImprovementNoReuse)
{
    double cost = optimizeDummyPolicy(10, 0, 0, 100);
    EXPECT_TRUE(cost < 0.1);
}

TEST(TestDummyPolicy, dummyPolicyImprovementReuse)
{
    double cost = optimizeDummyPolicy(10, 5, 0, 100);
    EXPECT_TRUE(cost < 0.1);
}

TEST(TestDummyPolicy, dummyPolicyImprovementReuseExtraRollout)
{
    double cost = optimizeDummyPolicy(10, 5, 1, 100);
    EXPECT_TRUE(cost < 0.1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
