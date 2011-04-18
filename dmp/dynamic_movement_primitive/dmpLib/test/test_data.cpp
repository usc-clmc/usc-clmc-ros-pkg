/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		test_data.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Feb 16, 2011

 *********************************************************************/

// system includes
#include <cmath>
#include <dmp_lib/logger.h>

// local includes
#include "test_data.h"

using namespace dmp_lib;
using namespace std;

namespace test_dmp
{

bool TestData::initialize(TestCase test_case)
{

  test_case_ = test_case;

  switch(test_case_)
  {

    case SIMPLE_TEST:
    {
      test_trajectory_file_name_.assign("test_circular_reaching");
      // test_trajectory_file_name_.assign("test_dmp_trajectory");

      variable_names_.clear();
      variable_names_.push_back("R_HAND_des_x");
      variable_names_.push_back("R_HAND_des_y");
    //  variable_names_.push_back("R_HAND_des_z");

      break;
    }

    case QUAT_TEST:
    {
      test_trajectory_file_name_.assign("test_dmp_trajectory");

      variable_names_.clear();
      variable_names_.push_back("R_HAND_des_x");
      variable_names_.push_back("R_HAND_des_y");
      variable_names_.push_back("R_HAND_des_z");
      quat_ts_index_ = 3;
      variable_names_.push_back("R_HAND_des_q0");
      variable_names_.push_back("R_HAND_des_q1");
      variable_names_.push_back("R_HAND_des_q2");
      variable_names_.push_back("R_HAND_des_q3");
      break;
    }
  }

  vector<double> goal_offset;
  goal_offset.resize(variable_names_.size());

  int num_tests = 2;
  goal_offsets_.resize(num_tests);
  error_thresholds_.resize(num_tests);
  duration_scales_.resize(num_tests);

  int test_id = 0;
  goal_offset[0] = 0.0;
  goal_offset[1] = 0.0;
//  goal_offset[2] = 0.0;
  goal_offsets_[test_id] = goal_offset;
  duration_scales_[test_id] = 1.0;
  error_thresholds_[test_id] = 0.5;
  test_id++;

  goal_offset[0] = 0.0;
  goal_offset[1] = 0.0;
//  goal_offset[2] = 0.5;
  goal_offsets_[test_id] = goal_offset;
  duration_scales_[test_id] = 1.0;
  error_thresholds_[test_id] = 0.5;
  test_id++;

  k_gain_ = 60.0;
  d_gain_ = 2.0 * sqrt(k_gain_);

  if(num_tests != test_id)
  {
    Logger::logPrintf("Number of tests does not match with test index.", Logger::FATAL);
    return false;
  }

  return (initialized_ = true);
}

bool TestData::getGoal(const int index, vector<double>& goal_offset) const
{
  if(!initialized_ || index < 0 || index >= (int)goal_offsets_.size())
  {
    return false;
  }
  goal_offset = goal_offsets_[index];
  return true;
}

bool TestData::getDurationScale(const int index, double& duration_scale) const
{
  if(!initialized_ || index < 0 || index >= (int)duration_scales_.size())
  {
    return false;
  }
  duration_scale = duration_scales_[index];
  return true;
}

bool TestData::getErrorThreshold(const int index, double& error_threshold) const
{
  if(!initialized_ || index < 0 || index >= (int)error_thresholds_.size())
  {
    return false;
  }
  error_threshold = error_thresholds_[index];
  return true;
}

int TestData::getNumTests() const
{
  return (int)goal_offsets_.size();
}

}
