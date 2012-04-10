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

  \file    accumulator.cpp

  \author  Peter Pastor
  \date    Jul 28, 2010

**********************************************************************/


// system includes
#include <algorithm>

// ros includes
#include <ros/ros.h>
#include <usc_utilities/assert.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

// local includes
#include <task_recorder/accumulator.h>

using namespace boost;
using namespace boost::accumulators;

namespace task_recorder
{

bool Accumulator::initialize(const int num_data_traces, const int num_samples)
{
    // if already initialized, only check whether num_data_traces and num_samples are correct
    if(initialized_)
    {
        ROS_ASSERT(num_data_traces_ == num_data_traces);
        ROS_ASSERT(num_samples_ == num_samples);
        return true;
    }

    num_data_traces_ = num_data_traces;
    num_samples_ = num_samples;

    std::vector<AccumulatorType> accumulator_vector;
    accumulator_vector.resize(num_samples_);
    accumulators_.resize(num_data_traces_, accumulator_vector);

    return (initialized_ = true);
}

bool Accumulator::add(const int data_trace_index, std::vector<double>& data_trace)
{
    ROS_ASSERT(initialized_);
    ROS_ASSERT(data_trace_index >= 0);
    ROS_ASSERT(data_trace_index < num_data_traces_);
    ROS_WARN_COND(num_samples_ != static_cast<int>(data_trace.size()), "num_samples_ (%i) == (%i) static_cast<int>(data_trace.size())",
                  num_samples_, static_cast<int>(data_trace.size()));
    ROS_ASSERT(num_samples_ == static_cast<int>(data_trace.size()));

    for (int i=0; i<static_cast<int>(data_trace.size()); ++i)
    {
//        if(data_trace_index==28)
//        {
//            ROS_INFO("Adding %i %f (%i)", data_trace_index, data_trace[i], i);
//        }
        accumulators_[data_trace_index][i](data_trace[i]);
    }
    return true;
}

bool Accumulator::getAccumulatedTrialStatistics(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics)
{
    accumulated_trial_statistics.clear();
    for(int j=0; j<num_samples_; ++j)
    {
        task_recorder::AccumulatedTrialStatistics accumulated_trial_statistic;
        accumulated_trial_statistic.mean.resize(num_data_traces_);
        accumulated_trial_statistic.variance.resize(num_data_traces_);
        for(int i=0; i<num_data_traces_; ++i)
        {
            accumulated_trial_statistic.count = extract_result< tag::count >( accumulators_[i][j] );
            accumulated_trial_statistic.mean[i] = extract_result< tag::mean >( accumulators_[i][j] );
            accumulated_trial_statistic.variance[i] = extract_result< tag::variance >( accumulators_[i][j] );
        }
        accumulated_trial_statistics.push_back(accumulated_trial_statistic);
    }
    return true;
}

}
