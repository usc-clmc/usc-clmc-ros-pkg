/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		test_trajectory.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 23, 2010

 *********************************************************************/

// system includes
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <dmp_lib/logger.h>

// local includes
#include "test_trajectory.h"

using namespace std;
using namespace dmp_lib;

// import most common Eigen types
using namespace Eigen;

namespace test_dmp
{

bool TestTrajectory::test(const string& filename, const TestData& testdata, const string base_directory)
{
  string data_directory_name = base_directory + "data/";
  std::string result_directory_name = base_directory + "result/";
  string prefix = ".clmc";

  // create result directory if it does not exist
  try
  {
    boost::filesystem::create_directories(boost::filesystem::path(result_directory_name));
  }
  catch (std::exception e)
  {
    dmp_lib::Logger::logPrintf("Could not create result directory: %s.", e.what());
    return false;
  }

  vector<string> variable_names = testdata.getTestVariableNames();
  vector<string> reordered_variable_names;
  for (int i = (int)variable_names.size() - 1; i >= 0; --i)
  {
    reordered_variable_names.push_back(variable_names[i]);
  }

  Trajectory pos_vel_acc_trajectory;
  string fname = data_directory_name + filename + prefix;
  if (!pos_vel_acc_trajectory.readFromCLMCFile(fname, variable_names))
  {
    dmp_lib::Logger::logPrintf("Could not read clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }

  fname.assign(result_directory_name + filename + string("_pos_vel_acc") + prefix);
  if (!pos_vel_acc_trajectory.writeToCLMCFile(fname))
  {
    dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }

  Trajectory pos_trajectory;
  fname.assign(data_directory_name + filename + prefix);
  if (!pos_trajectory.readFromCLMCFile(fname, variable_names, true))
  {
    dmp_lib::Logger::logPrintf("Could not read clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }
  fname.assign(result_directory_name + filename + string("_pos") + prefix);
  if (!pos_trajectory.writeToCLMCFile(fname, true))
  {
    dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }

  Trajectory pos_trajectory_copy;
  pos_trajectory_copy = pos_trajectory;
  fname.assign(result_directory_name + filename + string("_pos_vel_acc_copy") + prefix);
  if (!pos_trajectory_copy.writeToCLMCFile(fname, true))
  {
    dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }

  VectorXd nmse_vector = VectorXd::Zero(pos_trajectory.getDimension());
  if (!pos_trajectory.computePositionNMSE(pos_trajectory_copy, nmse_vector))
  {
    dmp_lib::Logger::logPrintf("Could not compute normalized mean squared error.", Logger::ERROR);
    return false;
  }

  double total_error = nmse_vector.sum();
  if (total_error > 0)
  {
    dmp_lib::Logger::logPrintf("Normalized mean squared of identical trajectories are >%f<.", Logger::ERROR, total_error);
    return false;
  }

  Trajectory pos_trajectory_copy_read;
  fname.assign(data_directory_name + filename + string("_pos_copy") + prefix);
  if (!pos_trajectory_copy_read.readFromCLMCFile(fname, variable_names, true))
  {
    dmp_lib::Logger::logPrintf("Could not read clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }
  if (!pos_trajectory_copy_read.computePositionNMSE(pos_trajectory, nmse_vector))
  {
    dmp_lib::Logger::logPrintf("Could not compute normalized mean squared error.", Logger::ERROR);
    return false;
  }

  total_error = nmse_vector.sum();
  if (total_error > 0)
  {
    dmp_lib::Logger::logPrintf("Normalized mean squared of identical trajectories are >%f<.", Logger::ERROR, total_error);
    return false;
  }

  Trajectory pos_vel_acc_trajectory_copy;
  pos_vel_acc_trajectory_copy = pos_vel_acc_trajectory;
  fname.assign(result_directory_name + filename + string("_pos_vel_acc_copy") + prefix);
  if (!pos_vel_acc_trajectory_copy.writeToCLMCFile(fname))
  {
    dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }

  if (!pos_vel_acc_trajectory.rearange(reordered_variable_names))
  {
    dmp_lib::Logger::logPrintf("Could not rearange trajectory.", Logger::ERROR);
    return false;
  }

  fname.assign(result_directory_name + filename + string("_pos_vel_acc_rearanged") + prefix);
  if (!pos_vel_acc_trajectory.writeToCLMCFile(fname))
  {
    dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }

  if (!pos_trajectory.rearange(reordered_variable_names))
  {
    dmp_lib::Logger::logPrintf("Could not rearange trajectory.", Logger::ERROR);
    return false;
  }

  fname.assign(result_directory_name + filename + string("_pos_rearanged") + prefix);
  if (!pos_trajectory.writeToCLMCFile(fname, true))
  {
    dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }


  Trajectory min_jerk_trajectory;
  VectorXd start = VectorXd::Zero(variable_names.size());
  VectorXd goal = VectorXd::Zero(variable_names.size());
  for (int i = 0; i < (int)variable_names.size(); ++i)
  {
    start(i) = 0.0;
    goal(i) = i;
  }
  double sampling_frequency = 300.0;
  int num_samples = 900;
  if(!min_jerk_trajectory.initializeWithMinJerk(variable_names, sampling_frequency, start, goal, num_samples))
  {
    dmp_lib::Logger::logPrintf("Could not initilaize trajectory with minimum jerk.", Logger::ERROR);
    return false;
  }
  fname.assign(result_directory_name + filename + string("_min_jerk") + prefix);
  if (!min_jerk_trajectory.writeToCLMCFile(fname))
  {
    dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", Logger::ERROR, fname.c_str());
    return false;
  }

  return true;
}

}

