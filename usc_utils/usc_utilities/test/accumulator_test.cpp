/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		accumulator_test.cpp

  \author	Peter Pastor
  \date		Mar 3, 2011

 *********************************************************************/

// system includes

// local includes
#include <gtest/gtest.h>
#include <usc_utilities/accumulator.h>
#include <usc_utilities/AccumulatedTrialStatistics.h>
#include <usc_utilities/param_server.h>

using namespace usc_utilities;

TEST(UscUtilitiesAccumulator, computeMean)
{

  int num_samples = 20;

  std::vector<double> data_trace_1;
  std::vector<double> data_trace_2;
  std::vector<double> data_trace_3;
  std::vector<double> accumulator_mean;
  std::vector<double> accumulator_variance;

  ros::NodeHandle node_handle("~");

  usc_utilities::read(node_handle, "accumulator_data_1", data_trace_1);
  usc_utilities::read(node_handle, "accumulator_data_2", data_trace_2);
  usc_utilities::read(node_handle, "accumulator_data_3", data_trace_3);

  usc_utilities::read(node_handle, "accumulator_mean", accumulator_mean);
  usc_utilities::read(node_handle, "accumulator_variance", accumulator_variance);

  EXPECT_EQ(static_cast<int>(data_trace_1.size()), num_samples);
  EXPECT_EQ(static_cast<int>(data_trace_2.size()), num_samples);
  EXPECT_EQ(static_cast<int>(data_trace_3.size()), num_samples);
  EXPECT_EQ(static_cast<int>(accumulator_mean.size()), num_samples);
  EXPECT_EQ(static_cast<int>(accumulator_variance.size()), num_samples);

  Accumulator accumulator;
  EXPECT_TRUE(accumulator.initialize(1, num_samples));
  EXPECT_TRUE(accumulator.add(0, data_trace_1));
  EXPECT_TRUE(accumulator.add(0, data_trace_2));
  EXPECT_TRUE(accumulator.add(0, data_trace_3));

  std::vector<usc_utilities::AccumulatedTrialStatistics> accumulated_trial_statistics;
  EXPECT_TRUE(accumulator.getAccumulatedTrialStatistics(accumulated_trial_statistics));

  EXPECT_EQ(static_cast<int>(accumulated_trial_statistics.size()), num_samples);
  for(int i=0; i<(int)accumulated_trial_statistics.size(); ++i)
  {
    for(int j=0; j<(int)accumulated_trial_statistics[i].mean.size(); ++j)
    {
      EXPECT_NEAR(accumulated_trial_statistics[i].mean[j], accumulator_mean[i], 1e-4);
      EXPECT_NEAR(accumulated_trial_statistics[i].variance[j], accumulator_variance[i], 1e-4);
    }
  }

}
