/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks              ...

  \file         interpolation.h

  \author       Peter Pastor
  \date         Jun 19, 2011

 *********************************************************************/

#ifndef INTERPOLATION_H_
#define INTERPOLATION_H_

// system includes
#include <vector>
#include <bspline/BSpline.h>

#include <roscpp_utilities/assert.h>

// local includes
#include <task_recorder2_msgs/definitions.h>

namespace task_recorder2
{

/*!
 * @param input_vector
 * @param target_vector
 * @param cutoff_wave_length
 * @param input_querry
 * @param output_vector
 * @param compute_slope
 * @return True on success, otherwise False
 */
bool resampleBSpline(const std::vector<double>& input_vector,
                     const std::vector<double>& target_vector,
                     const double cutoff_wave_length,
                     const std::vector<double>& input_querry,
                     std::vector<double>& output_vector,
                     bool compute_slope);

/**
 * Given input samples of input_y = f(input_x), calculates output_y = f(output_x) using linear interpolation
 * Assumes that input_x and output_x are sorted!
 * @param input_x
 * @param input_y
 * @param output_x
 * @param output_y
 * @return True on success, otherwise False
 */
bool resampleLinearNoBounds(const std::vector<double>& input_x,
                            const std::vector<double>& input_y,
                            const std::vector<double>& output_x,
                            std::vector<double>& output_y);

/// inline functions follow

inline bool resampleLinearNoBounds(const std::vector<double>& input_x,
                                   const std::vector<double>& input_y,
                                   const std::vector<double>& output_x,
                                   std::vector<double>& output_y)
{
  ROS_ASSERT(input_x.size() == input_y.size());
  ROS_ASSERT(input_x.size() > 1);

  const unsigned int NUM_OUTPUTS = output_x.size();
  output_y.resize(NUM_OUTPUTS, 0.0);
  const unsigned int NUM_INPUTS = input_x.size();
  unsigned int input_index = 0;
  for (unsigned int i = 0; i < NUM_OUTPUTS; ++i)
  {
    if (output_x[i] < input_x[0])
    {
      output_y[i] = input_y[0];
    }
    else if (output_x[i] > input_x[NUM_INPUTS - 1])
    {
      output_y[i] = input_y[NUM_INPUTS - 1];
    }
    else
    {
      while (input_x[input_index + 1] < output_x[i] && input_index < input_x.size() - 1)
      {
        input_index++;
      }
      if (input_index >= input_x.size() - 1)
      {
        ROS_ERROR("Invalid input. Cannot compute linear interpolation.");
        return false;
      }
      // previous version
      // double delta = input_x[input_index+1] - input_x[input_index];
      // double delta_after = (output_x[i]-input_x[input_index])/delta;
      // double delta_before = (input_x[input_index+1] - output_x[i])/delta;
      // output_y[i] = delta_before*input_y[input_index] + delta_after*input_y[input_index+1];
      // solution by schorfi: handling it like a straight line (more efficient: 2 multiplications)
      // determining the slope between the adjacent points
      if (input_x[input_index + 1] <= input_x[input_index])
      {
        ROS_ERROR("Input invalid. Time stamp at >%i< is >%f< and time stamp at >%i< is >%f<.",
                  (int)input_index + 1, input_x[input_index + 1], (int)input_index, input_x[input_index]);
        return false;
      }
      double slope = (input_y[input_index + 1] - input_y[input_index]) / (input_x[input_index + 1] - input_x[input_index]);
      // evaluate function f(x) = slope * x + y_0 (assuming input_xy[input_index] to be (0,0)
      output_y[i] = (output_x[i] - input_x[input_index]) * slope + input_y[input_index];
    }
  }
  return true;
}

inline bool resampleBSpline(const std::vector<double>& input_vector,
                            const std::vector<double>& target_vector,
                            const double cutoff_wave_length,
                            const std::vector<double>& input_querry,
                            std::vector<double>& output_vector,
                            bool compute_slope)
{
  ROS_ASSERT_MSG(!input_vector.empty(), "Input vector is empty. Cannot re-sample trajectory using a B-spline.");
  ROS_ASSERT_MSG(!input_querry.empty(), "Input query is empty. Cannot re-sample trajectory using a B-spline.");
  if (input_vector.size() != target_vector.size())
    return false;

  std::vector<double> tmp_input_vector = input_vector;
  std::vector<double> tmp_target_vector = target_vector;
  std::vector<int> indexes;
  unsigned int invalid_data_counter = 0;
  for (unsigned int i = 0; i < tmp_input_vector.size(); ++i)
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
  if (invalid_data_counter > tmp_input_vector.size())
  {
    ROS_WARN("Found >%i< invalid data points when re-sampling the trajectory.", invalid_data_counter);
  }

  const unsigned int NUM_ROWS = target_vector.size();
  double x_vector[NUM_ROWS];
  double y_vector[NUM_ROWS];
  for (unsigned int i = 0; i < NUM_ROWS; ++i)
  {
    x_vector[i] = tmp_input_vector[i];
    y_vector[i] = tmp_target_vector[i];
  }

  const unsigned int NUM_SAMPLES = input_querry.size();
  output_vector.clear();
  output_vector.resize(NUM_SAMPLES, 0.0);

  BSpline<double>::Debug(0);
  BSpline<double> b_spline(&(x_vector[0]), NUM_ROWS, &(y_vector[0]), cutoff_wave_length); //, BSplineBase<double>::BC_ZERO_SECOND);
  if (b_spline.ok())
  {
    for (unsigned int s = 0; s < NUM_SAMPLES; ++s)
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
    ROS_ERROR("Could not create b-spline.");
    return false;
  }
  return true;
}

}

#endif /* INTERPOLATION_H_ */
