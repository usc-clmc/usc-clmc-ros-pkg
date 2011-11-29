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

 \file    bspline.h

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

#ifndef UTILITIES_BSPLINE_H_
#define UTILITIES_BSPLINE_H_

// system includes
#include <vector>

// ros includes
#include <bspline/BSpline.h>

#include <sensor_msgs/JointState.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

// local includes
#include <usc_utilities/assert.h>
#include <usc_utilities/logging.h>

namespace usc_utilities
{

bool computeVelocities(const std::vector<double>& positions,
                       const double mean_dt,
                       std::vector<double>& velocities);
bool computeFilteredVelocities(const std::vector<double>& positions,
                               const double mean_dt,
                               std::vector<double>& velocities);

/*!
 * @param input_vector
 * @param target_vector
 * @param cutoff_wave_length
 * @param input_querry
 * @param output_vector
 * @param compute_slope
 * @param verbose
 * @return True on success, otherwise False
 */
bool resample(const std::vector<double>& input_vector,
              const std::vector<double>& target_vector,
              const double cutoff_wave_length,
              const std::vector<double>& input_querry,
              std::vector<double>& output_vector,
              bool compute_slope,
              bool verbose = false);

/**
 * Given input samples of input_y = f(input_x), calculates output_y = f(output_x) using linear interpolation
 * Assumes that input_x and output_x are sorted!
 */
bool resampleLinearNoBounds(const std::vector<double>& input_x,
                            const std::vector<double>& input_y,
                            const std::vector<double>& output_x,
                            std::vector<double>& output_y);

/**
 * Given input samples of input_y = f(input_x), calculates output_y = f(output_x) using linear interpolation
 * Assumes that input_x and output_x are sorted!
 * Assumes that the output_x query points are entirely contained within input_x.
 */
bool resampleLinear(const std::vector<double>& input_x,
                     const std::vector<double>& input_y,
                     const std::vector<double>& output_x,
                     std::vector<double>& output_y);
////// inline functions follow ///////////////

inline bool computeVelocities(const std::vector<double>& positions,
                              const double mean_dt,
                              std::vector<double>& velocities)
{
  ROS_VERIFY(positions.size() > 1);
  ROS_VERIFY(mean_dt > 0);

  velocities.clear();
  velocities.resize(positions.size());
  for (int i = 0; i < static_cast<int> (positions.size()) - 1; i++)
  {
    velocities[i] = (positions[i + 1] - positions[i]) / mean_dt;
  }
  velocities[positions.size() - 1] = velocities[positions.size() - 2];
  return true;
}

inline bool computeFilteredVelocities(const std::vector<double>& positions,
                                      const double mean_dt,
                                      std::vector<double>& velocities)
{
  ROS_VERIFY(positions.size() > 1);
  ROS_VERIFY(mean_dt > 0);

  std::vector<double> tmp_positions;
  tmp_positions.push_back(positions.front());
  tmp_positions.push_back(positions.front());
  tmp_positions.insert(tmp_positions.end(), positions.begin(), positions.end());
  tmp_positions.push_back(positions.back());
  tmp_positions.push_back(positions.back());

  velocities.clear();
  velocities.resize(positions.size());
  for (int i = 1; i < static_cast<int> (velocities.size() - 1); ++i)
  {
    velocities[i] = (tmp_positions[i] - (static_cast<double> (8.0) * tmp_positions[i + 1]) + (static_cast<double> (8.0) * tmp_positions[i + 3])
        - tmp_positions[i + 4]) / static_cast<double> (12.0);
    velocities[i] /= mean_dt;
  }
  velocities.front() = velocities[1];
  velocities.back() = velocities[velocities.size() - 2];
  return true;
}

//inline bool removeInvalidData(std::vector<double>& input_vector, std::vector<double>& target_vector)
//{
//    ROS_WARN("Removing invalid data...");
//    std::vector<int> indexes;
//    for (int i = 0; i < static_cast<int> (input_vector.size()); i++)
//    {
//        if(input_vector[i] < 1e-6)
//        {
//            ROS_WARN("Invalid data point detected with index %i.", i);
//            indexes.push_back(i);
//        }
//    }
//    for( std::vector<int>::reverse_iterator rit = indexes.rbegin(); rit != indexes.rend(); ++rit)
//    {
//        ROS_WARN("Removing invalid data point with index %i.", *rit);
//        input_vector.erase(input_vector.begin() + *rit);
//        target_vector.erase(target_vector.begin() + *rit);
//    }
//    return true;
//}

inline bool resample(const std::vector<double>& input_vector,
                     const std::vector<double>& target_vector,
                     const double cutoff_wave_length,
                     const std::vector<double>& input_querry,
                     std::vector<double>& output_vector,
                     bool compute_slope,
                     bool verbose)
{
  ROS_ASSERT_MSG(!input_vector.empty(), "Input vector is empty. Cannot resample trajecoty using a bspline.");
  ROS_ASSERT_MSG(!input_querry.empty(), "Input querry is empty. Cannot resample trajecoty using a bspline.");
  ROS_VERIFY(input_vector.size() == target_vector.size());

  const int num_rows = static_cast<int> (target_vector.size());

  // ROS_VERIFY(removeInvalidData(input_vector, target_vector));
  // ###################################################################
  std::vector<double> tmp_input_vector = input_vector;
  std::vector<double> tmp_target_vector = target_vector;
  std::vector<int> indexes;
  int invalid_data_counter = 0;
  for (int i = 0; i < static_cast<int> (tmp_input_vector.size()); i++)
  {
    if (tmp_input_vector[i] < 1e-6)
    {
      invalid_data_counter++;
      // ROS_WARN("Invalid data point >%f< detected with index >%i<.", tmp_input_vector[i], i);
      indexes.push_back(i);
    }
  }
  for (std::vector<int>::reverse_iterator rit = indexes.rbegin(); rit != indexes.rend(); ++rit)
  {
    // ROS_WARN("Removing invalid data point with index %i.", *rit);
    tmp_input_vector.erase(tmp_input_vector.begin() + *rit);
    tmp_target_vector.erase(tmp_target_vector.begin() + *rit);
  }

  if(invalid_data_counter > (int)tmp_input_vector.size())
  {
    ROS_WARN("Found >%i< invalid data points when resampling the trajectory.", invalid_data_counter);
  }
  // ###################################################################

  double x_vector[num_rows];
  double y_vector[num_rows];
  for (int i = 0; i < num_rows; ++i)
  {
    x_vector[i] = tmp_input_vector[i];
    y_vector[i] = tmp_target_vector[i];
  }

  int num_samples = static_cast<int> (input_querry.size());
  output_vector.clear();
  output_vector.resize(num_samples);

  BSpline<double>::Debug(0);
  BSpline<double> b_spline(&(x_vector[0]), num_rows, &(y_vector[0]), cutoff_wave_length); //, BSplineBase<double>::BC_ZERO_SECOND);
  if (b_spline.ok())
  {
    for (int s = 0; s < num_samples; ++s)
    {
      if (compute_slope)
      {
        output_vector[s] = b_spline.slope(input_querry[s]);
      }
      else
      {
        output_vector[s] = b_spline.evaluate(input_querry[s]);
      }
    }
  }
  else
  {
    // if (verbose)
    {
      ROS_ERROR("Could not create b-spline.");
      log(tmp_input_vector, "/tmp/bspline_input.txt");
      log(tmp_target_vector, "/tmp/bspline_target.txt");
      log(input_querry, "/tmp/bspline_querry.txt");
      ROS_ERROR("Number of input values is >%i<.", (int)tmp_input_vector.size());
      ROS_ERROR("Number of target values is >%i<.", (int)tmp_target_vector.size());
      ROS_ERROR("Number of rows is >%i<.", num_rows);
      ROS_ERROR("Cuttoff is >%f<.", cutoff_wave_length);
      try
      {
        ros::Time::init();
        rosbag::Bag bag(std::string("/tmp/bspline_failed.bag"), rosbag::bagmode::Write);
        sensor_msgs::JointState fake_joint_state;
        fake_joint_state.position = tmp_input_vector;
        fake_joint_state.velocity = tmp_target_vector;
        fake_joint_state.effort = input_querry;
        bag.write("/joint_states", ros::Time::now(), fake_joint_state);
        bag.close();
      }
      catch (rosbag::BagIOException ex)
      {
        ROS_ERROR("Could not open bag file /tmp/bspline_failed.bag: %s", ex.what());
        return false;
      }
    }
    return false;
  }
  return true;
}

//inline bool resample(const std::vector<ros::Time>& time_stamps,
//                     const std::vector<std::vector<double> >& values,
//                     const int num_samples,
//                     const double cutoff_wave_length,
//                     std::vector<std::vector<double> >& resampled_values)
//{
//  ROS_ASSERT_MSG(false, "This function not implemented!!");
//  return true;
//}

inline bool resampleLinearNoBounds(const std::vector<double>& input_x,
                                   const std::vector<double>& input_y,
                                   const std::vector<double>& output_x,
                                   std::vector<double>& output_y)
{
  ROS_ASSERT(input_x.size() == input_y.size());
  ROS_ASSERT(input_x.size() > 1);

  const int num_outputs = output_x.size();
  output_y.resize(num_outputs);
  const int num_inputs = input_x.size();

  unsigned int input_index = 0;
  for (int i = 0; i < num_outputs; ++i)
  {
    if(output_x[i] < input_x[0])
    {
      output_y[i] = input_y[0];
    }
    else if (output_x[i] > input_x[num_inputs-1])
    {
      output_y[i] = input_y[num_inputs-1];
    }
    else
    {
      while (input_x[input_index + 1] < output_x[i] && input_index < input_x.size() - 1)
      {
        input_index++;
        // ROS_INFO("index = %i", input_index);
      }
      ROS_ASSERT(input_index < input_x.size()-1);

      // previous version
      // double delta = input_x[input_index+1] - input_x[input_index];
      // double delta_after = (output_x[i]-input_x[input_index])/delta;
      // double delta_before = (input_x[input_index+1] - output_x[i])/delta;
      // output_y[i] = delta_before*input_y[input_index] + delta_after*input_y[input_index+1];
      // solution by schorfi: handling it like a straight line (more efficient: 2 multiplications)
      // determining the slope between the adjacent points
      ROS_ASSERT_MSG(input_x[input_index + 1] > input_x[input_index], "Input invalid. Time stamp at >%i< is >%f< and time stamp at >%i< is >%f<.",
                     input_index + 1, input_x[input_index + 1], input_index, input_x[input_index]);
      double slope = (input_y[input_index + 1]-input_y[input_index])/(input_x[input_index + 1]-input_x[input_index]);
      // evaluate function f(x) = slope*x + y_0 (assuming input_xy[input_index] to be (0,0)
      output_y[i] = (output_x[i]-input_x[input_index])*slope + input_y[input_index];
    }
  }
  return true;
}

inline bool resampleLinear(const std::vector<double>& input_x,
                           const std::vector<double>& input_y,
                           const std::vector<double>& output_x,
                           std::vector<double>& output_y)
{
  ROS_ASSERT(input_x.size() == input_y.size());
  ROS_ASSERT(output_x.front() > input_x.front());
  ROS_ASSERT(output_x.back() < input_x.back());

  int num_outputs = output_x.size();
  output_y.resize(num_outputs);

  unsigned int input_index = 0;
  for (int i = 0; i < num_outputs; ++i)
  {
    while (input_x[input_index+1] < output_x[i] && input_index < input_x.size()-1)
      input_index++;
    ROS_ASSERT(input_index < input_x.size()-1);
    // previous version
    // double delta = input_x[input_index+1] - input_x[input_index];
    // double delta_after = (output_x[i]-input_x[input_index])/delta;
    // double delta_before = (input_x[input_index+1] - output_x[i])/delta;
    // output_y[i] = delta_before*input_y[input_index] + delta_after*input_y[input_index+1];
    // solution by schorfi: handling it like a straight line (more efficient: 2 multiplications)
    // determining the slope between the adjacent points
    ROS_ASSERT_MSG(input_x[input_index + 1] > input_x[input_index], "Input invalid. Time stamp at >%i< is >%f< and time stamp at >%i< is >%f<.",
                   input_index + 1, input_x[input_index + 1], input_index, input_x[input_index]);
    double slope = (input_y[input_index + 1]-input_y[input_index])/(input_x[input_index + 1]-input_x[input_index]);
    // evaluate function f(x) = slope*x + y_0 (assuming input_xy[input_index] to be (0,0)
    output_y[i] = (output_x[i]-input_x[input_index])*slope + input_y[input_index];
  }
  return true;
}

}

#endif /* UTILITIES_BSPLINE_H_ */
