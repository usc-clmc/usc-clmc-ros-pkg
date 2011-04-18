/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks    ...

 \file   accumulator.cpp

 \author Peter Pastor
 \date   Nov 3, 2010

 *********************************************************************/

// system includes
#include <algorithm>

// ros includes
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

// local includes
#include <usc_utilities/assert.h>
#include <usc_utilities/accumulator.h>

using namespace boost;
using namespace boost::accumulators;

namespace usc_utilities
{

bool Accumulator::initialize(const int num_data_traces,
                             const int num_samples)
{
  // if already initialized, only check whether num_data_traces and num_samples are correct
  if (initialized_)
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

bool Accumulator::add(const int data_trace_index,
                      std::vector<double>& data_trace)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(data_trace_index >= 0);
  ROS_ASSERT(data_trace_index < num_data_traces_);
  ROS_WARN_COND(num_samples_ != static_cast<int>(data_trace.size()), "num_samples_ (%i) == (%i) static_cast<int>(data_trace.size())",
      num_samples_, static_cast<int>(data_trace.size()));
  ROS_ASSERT(num_samples_ == static_cast<int>(data_trace.size()));

  for (int i = 0; i < static_cast<int> (data_trace.size()); ++i)
  {
    accumulators_[data_trace_index][i](data_trace[i]);
  }
  return true;
}

bool Accumulator::getAccumulatedTrialStatistics(std::vector<usc_utilities::AccumulatedTrialStatistics>& accumulated_trial_statistics)
{
  accumulated_trial_statistics.clear();
  for (int j = 0; j < num_samples_; ++j)
  {
    usc_utilities::AccumulatedTrialStatistics accumulated_trial_statistic;
    accumulated_trial_statistic.mean.resize(num_data_traces_);
    accumulated_trial_statistic.variance.resize(num_data_traces_);
    for (int i = 0; i < num_data_traces_; ++i)
    {
      accumulated_trial_statistic.count = extract_result<tag::count> (accumulators_[i][j]);
      accumulated_trial_statistic.mean[i] = extract_result<tag::mean> (accumulators_[i][j]);
      accumulated_trial_statistic.variance[i] = extract_result<tag::variance> (accumulators_[i][j]);
    }
    accumulated_trial_statistics.push_back(accumulated_trial_statistic);
  }
  return true;
}

}
