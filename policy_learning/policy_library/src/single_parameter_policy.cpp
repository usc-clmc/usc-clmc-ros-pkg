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
#include <boost/foreach.hpp>

// ros includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <policy_library/single_parameter_policy.h>


// import most common Eigen types
using namespace Eigen;

namespace policy_library
{

static const char* SINGLE_PARAMETER_POLICY_TOPIC_NAME = "single_parameter_policy";

SingleParameterPolicy::SingleParameterPolicy()
    : initialized_(false)
{
}

SingleParameterPolicy::~SingleParameterPolicy()
{
}

bool SingleParameterPolicy::initialize(int num_dimensions)
{
    num_dimensions_ = num_dimensions;
    parameters_.resize(num_dimensions, VectorXd::Zero(1));
    //library_directory_name_.assign("/tmp");
    return (initialized_ = true);
}

bool SingleParameterPolicy::setNumTimeSteps(const int num_time_steps)
{
    ROS_ASSERT(initialized_);
    num_time_steps_ = num_time_steps;
    return true;
}

bool SingleParameterPolicy::getNumTimeSteps(int& num_time_steps)
{
    ROS_ASSERT(initialized_);
    num_time_steps = num_time_steps_;
    return true;
}

bool SingleParameterPolicy::getNumDimensions(int& num_dimensions)
{
    ROS_ASSERT(initialized_);
    num_dimensions = num_dimensions_;
    return true;
}

bool SingleParameterPolicy::getNumParameters(std::vector<int>& num_params)
{
    ROS_ASSERT(initialized_);
    num_params.clear();
    num_params.resize(num_dimensions_, 1);
    return true;
}

bool SingleParameterPolicy::getBasisFunctions(std::vector<MatrixXd>& basis_functions)
{
    ROS_ASSERT(initialized_);
    basis_functions.clear();
    basis_functions.resize(num_dimensions_, MatrixXd::Ones(num_time_steps_, 1));
    return true;
}

bool SingleParameterPolicy::getControlCosts(std::vector<MatrixXd>& control_costs)
{
    control_costs.clear();
    control_costs.resize(num_dimensions_, MatrixXd::Ones(1,1));
    return true;
}

bool SingleParameterPolicy::updateParameters(const std::vector<MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights)
{
    ROS_ASSERT(initialized_);
    // average the updates with a weight depending on number of time-steps left in the trajectory
    for (int d=0; d<num_dimensions_; ++d)
    {
        double update = 0.0;
        double update_denom = 0.0;
        for (int t=0; t<num_time_steps_; ++t)
        {
            double weight = double(num_time_steps_-t);
            update_denom += weight;
            update += weight * updates[d](t,0);
        }
        update /= update_denom;
        parameters_[d](0) += update;
    }
    return true;
}

bool SingleParameterPolicy::getParameters(std::vector<VectorXd>& parameters)
{
    ROS_ASSERT(initialized_);
    parameters = parameters_;
    return true;
}

bool SingleParameterPolicy::setParameters(const std::vector<VectorXd>& parameters)
{
    ROS_ASSERT(initialized_);
    parameters_ = parameters;
    return true;
}

std::string SingleParameterPolicy::getClassName()
{
    return "SingleParameterPolicy";
}

bool SingleParameterPolicy::readFromFile(const std::string& abs_bagfile_name)
{
    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(SINGLE_PARAMETER_POLICY_TOPIC_NAME));
        int message_counter = 0;
        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            if (message_counter > 1)
            {
                ROS_ERROR("Bagfile should only contain a single single parameter policy.");
                return false;
            }
            message_counter++;

            policy_msgs::SingleParameterPolicy::ConstPtr single_parameter_policy = msg.instantiate<policy_msgs::SingleParameterPolicy> ();
            ROS_ASSERT(single_parameter_policy != NULL);
            if (!initFromMessage(*single_parameter_policy))
            {
                ROS_ERROR("Could not get and initialize the single parameter policy.");
                return false;
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }
    return true;
}

bool SingleParameterPolicy::writeToFile(const std::string& abs_bagfile_name)
{
    ROS_ASSERT(initialized_);
    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Write);
        policy_msgs::SingleParameterPolicy single_parameter_policy;
        if (!writeToMessage(single_parameter_policy))
        {
            ROS_ERROR("Could not store single parameter policy into a message.");
            return false;
        }
        bag.write(SINGLE_PARAMETER_POLICY_TOPIC_NAME, ros::Time::now(), single_parameter_policy);
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }
    return true;
}

bool SingleParameterPolicy::initFromMessage(const policy_msgs::SingleParameterPolicy& spp_msg)
{
    initialized_ = spp_msg.initialized;
    num_dimensions_ = spp_msg.num_dimensions;
    num_time_steps_ = spp_msg.num_time_steps;
    std::vector<double> parameters = spp_msg.parameters;
    ROS_ASSERT(static_cast<int>(parameters.size()) == num_dimensions_);
    parameters_.clear();
    for (int i=0; i<static_cast<int>(parameters.size()); ++i)
    {
        VectorXd parameter = VectorXd::Zero(1);
        parameter(0) = parameters[i];
        parameters_.push_back(parameter);
    }
    //item_id_ = spp_msg.id;
    //library_directory_name_.assign(spp_msg.library_directory_name);
    return true;
}

bool SingleParameterPolicy::writeToMessage(policy_msgs::SingleParameterPolicy& spp_msg)
{
    spp_msg.initialized = initialized_;
    spp_msg.num_dimensions = num_dimensions_;
    spp_msg.num_time_steps = num_time_steps_;

    if(static_cast<int>(parameters_.size()) != num_dimensions_)
    {
        ROS_ERROR("parameters_.size() = %i != %i = num_dimensions_", static_cast<int>(parameters_.size()), num_dimensions_);
        ROS_ASSERT(static_cast<int>(parameters_.size()) == num_dimensions_);
    }

    spp_msg.parameters.clear();
    for (int i=0; i<static_cast<int>(parameters_.size()); ++i)
    {
        spp_msg.parameters.push_back(parameters_[i](0));
    }

    // ROS_INFO("writing id = %i and directory = %s", item_id_, library_directory_name_.c_str());
    //spp_msg.id = item_id_;

//    if(library_directory_name_.empty())
//    {
//        // HACK
//        library_directory_name_.assign("/tmp/mixed_policy");
//    }
//    spp_msg.library_directory_name.assign(library_directory_name_);
    return true;
}

}
