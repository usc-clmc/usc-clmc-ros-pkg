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

// system includes

// ros includes
#include <ros/ros.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <policy_library/mixed_policy.h>

namespace policy_library
{

MixedPolicy::MixedPolicy() :
    initialized_(false)
{
}

MixedPolicy::~MixedPolicy()
{
}

bool MixedPolicy::initialize(std::vector<boost::shared_ptr<Policy> >& policies)
{
    policies_ = policies;
    num_policies_ = policies.size();
    num_dimensions_ = 0;
    num_dimensions_per_policy_.clear();
    for (int p=0; p<num_policies_; ++p)
    {
        int num_dim;
        ROS_VERIFY(policies[p]->getNumDimensions(num_dim));
        num_dimensions_per_policy_.push_back(num_dim);
        num_dimensions_ += num_dim;
    }
    ROS_INFO("Initializing mixed policy with %i dimensions.", num_dimensions_);
    return (initialized_ = true);
}

bool MixedPolicy::setNumTimeSteps(const int num_time_steps)
{
    ROS_ASSERT(initialized_);
    num_time_steps_ = num_time_steps;
    return true;
}

bool MixedPolicy::getNumTimeSteps(int& num_time_steps)
{
    ROS_ASSERT(initialized_);
    num_time_steps = num_time_steps_;
    return true;
}

bool MixedPolicy::getNumDimensions(int& num_dimensions)
{
    ROS_ASSERT(initialized_);
    num_dimensions = num_dimensions_;
    return true;
}

bool MixedPolicy::getNumParameters(std::vector<int>& num_params)
{
    ROS_ASSERT(initialized_);
    num_params.clear();
    std::vector<int> num_p;
    for (int p=0; p<num_policies_; ++p)
    {
        num_p.clear();
        policies_[p]->getNumParameters(num_p);
        num_params.insert(num_params.end(), num_p.begin(), num_p.end());
    }
    return true;
}

bool MixedPolicy::getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions)
{
    ROS_ASSERT(initialized_);
    basis_functions.clear();
    std::vector<Eigen::MatrixXd> basis_f;
    for (int p=0; p<num_policies_; ++p)
    {
        basis_f.clear();
        policies_[p]->getBasisFunctions(basis_f);
        basis_functions.insert(basis_functions.end(), basis_f.begin(), basis_f.end());
    }
    return true;
}

bool MixedPolicy::getControlCosts(std::vector<Eigen::MatrixXd>& control_costs)
{
    ROS_ASSERT(initialized_);
    control_costs.clear();
    std::vector<Eigen::MatrixXd> control_c;
    for (int p=0; p<num_policies_; ++p)
    {
        control_c.clear();
        policies_[p]->getControlCosts(control_c);
        control_costs.insert(control_costs.end(), control_c.begin(), control_c.end());
    }
    return true;
}

bool MixedPolicy::updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights)
{
    int start_dimension = 0;
    for (int p=0; p<num_policies_; ++p)
    {
        std::vector<Eigen::MatrixXd> p_updates;
        p_updates.insert(p_updates.begin(), updates.begin()+start_dimension, updates.begin()+start_dimension+num_dimensions_per_policy_[p]);
        policies_[p]->updateParameters(p_updates, time_step_weights);
        start_dimension += num_dimensions_per_policy_[p];
    }
    return true;
}

bool MixedPolicy::getParameters(std::vector<Eigen::VectorXd>& parameters)
{
    ROS_ASSERT(initialized_);
    parameters.clear();
    for (int p=0; p<num_policies_; ++p)
    {
        std::vector<Eigen::VectorXd> params;
        policies_[p]->getParameters(params);
        parameters.insert(parameters.end(), params.begin(), params.end());
    }
    return true;
}

bool MixedPolicy::setParameters(const std::vector<Eigen::VectorXd>& parameters)
{
    ROS_ASSERT(initialized_);
    int start_dimension = 0;
    for (int p=0; p<num_policies_; ++p)
    {
        std::vector<Eigen::VectorXd> params;
        ROS_INFO("MixedPolicy: policy %i has %i dimensions.", p, num_dimensions_per_policy_[p]);
        params.insert(params.begin(), parameters.begin()+start_dimension, parameters.begin()+start_dimension+num_dimensions_per_policy_[p]);
        policies_[p]->setParameters(params);
        start_dimension += num_dimensions_per_policy_[p];
    }
    return true;
}

bool MixedPolicy::readFromFile(const std::string& directory_name)
{
    ROS_ASSERT(initialized_);
    // ROS_WARN("MixedPolicy::readFromDisc(%s)", directory_name.c_str());

    std::string dir_name;
    dir_name = directory_name;
    usc_utilities::appendTrailingSlash(dir_name);
    for(int i=0; i<(int)policies_.size(); ++i)
    {
        std::string bag_file_name;
        bag_file_name.assign(dir_name + policies_[i]->getClassName() + std::string("_"));
        bag_file_name.append(usc_utilities::getString(i) + std::string("_"));
        bag_file_name.append(usc_utilities::getString(trial_id_) + std::string(".bag"));
        ROS_WARN("policies_[%i]->readFromDisc(%s)", i, bag_file_name.c_str());
        ROS_VERIFY(policies_[i]->readFromFile(bag_file_name));
    }
    return true;
}

bool MixedPolicy::writeToFile(const std::string& directory_name)
{
    ROS_ASSERT(initialized_);
    // ROS_WARN("MixedPolicy::writeToDisc(%s)", directory_name.c_str());

    size_t rollout_separater_pos = directory_name.find_last_of("_rollout_");
    size_t bag_separater_pos = directory_name.find_last_of(".bag");
    if((rollout_separater_pos!=std::string::npos) && (bag_separater_pos!=std::string::npos))
    {
        // it is a rollout
        // HACK
        std::string rollout_string = directory_name.substr(rollout_separater_pos+1, 1);
        //ROS_INFO("rollout_file_name = %s", rollout_string.c_str());

        std::stringstream ss(rollout_string);
        int rollout_id;
        ss >> rollout_id;

        size_t separater_pos = directory_name.find_last_of("/");
        ROS_ASSERT(separater_pos!=std::string::npos);
        std::string directory = directory_name.substr(0, separater_pos);
        usc_utilities::appendTrailingSlash(directory);

        //        if (!boost::filesystem::exists(directory))
//        {
//            ROS_INFO("Creating directory %s...", directory.c_str());
//            ROS_VERIFY(boost::filesystem::create_directories(directory));
//        }
        // std::string directory = directory_name.substr(0, )

        std::string abs_file_name = directory_name.substr(separater_pos+1, bag_separater_pos-3);
         for(int i=0; i<(int)policies_.size(); ++i)
        {
            abs_file_name.assign(directory + policies_[i]->getClassName() + std::string("_"));
            abs_file_name.append(usc_utilities::getString(i) + std::string("_"));
            abs_file_name.append(usc_utilities::getString(trial_id_));
            abs_file_name.append(std::string("_rollout_") + usc_utilities::getString(rollout_id) + std::string(".bag"));
            // ROS_INFO("directory = %s", directory.c_str());
            // ROS_INFO("abs_file_name = %s", abs_file_name.c_str());
            ROS_VERIFY(policies_[i]->writeToFile(abs_file_name));
        }
        return true;
    }

    std::string dir_name;
    dir_name = directory_name;
    usc_utilities::appendTrailingSlash(dir_name);
    for(int i=0; i<(int)policies_.size(); ++i)
    {
        std::string bag_file_name;
        bag_file_name.assign(dir_name + policies_[i]->getClassName() + std::string("_"));
        bag_file_name.append(usc_utilities::getString(i) + std::string("_"));
        bag_file_name.append(usc_utilities::getString(trial_id_) + std::string(".bag"));
        // ROS_WARN("policies_[%i]->writeToDisc(%s)", i, bag_file_name.c_str());
        ROS_VERIFY(policies_[i]->writeToFile(bag_file_name));
    }
    return true;
}

std::string MixedPolicy::getClassName()
{
  return "MixedPolicy";
}

}
