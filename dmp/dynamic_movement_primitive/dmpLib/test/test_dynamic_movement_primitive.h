/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		test_dynamic_movement_primitive.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 23, 2010

 *********************************************************************/

#ifndef TEST_DYNAMIC_MOVEMENT_PRIMITIVE_H_
#define TEST_DYNAMIC_MOVEMENT_PRIMITIVE_H_

// system includes
#include <string>
#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Eigen>
#include <Eigen/Core>

#include <dmp_lib/dynamic_movement_primitive.h>
#include <dmp_lib/trajectory.h>
#include <dmp_lib/logger.h>

namespace test_dmp
{

static std::string prefix = ".clmc";
static std::string test_target = "_test_target";
static std::string test_min_jerk_target = "_test_min_jerk_target";

/*!
 */
template<class DMPType>

  class TestDMP
  {

  public:

    /*!
     * @param dmp
     * @param base_filename
     * @param extension
     * @param num_data_points
     * @param goal_offset
     * @param duration_scale
     * @param error_threshold
     * @param index
     * @param generate_test_data
     * @return
     */
  static bool test(DMPType& dmp,
                   const std::string& base_filename,
                   const std::string& extension,
                   const int num_data_points,
                   const std::vector<double> goal_offset,
                   const double duration_scale,
                   const double error_threshold,
                   const int index,
                   const std::string base_directory = "",
                   bool generate_test_data = false);

  private:

    /*!
     */
    TestDMP() {};
    virtual ~TestDMP() {};

    static bool testLearningFromTrajectory(DMPType& dmp,
                                           const std::string& base_filename,
                                           const std::string& extension,
                                           // const int num_data_points,
                                           const std::vector<double> goal_offset,
                                           const double duration_scale,
                                           const double error_threshold,
                                           const int index,
                                           const std::string base_directory = "",
                                           bool generate_test_data = false);

    static bool testLearningFromMinJerk(DMPType& dmp,
                                        const std::string& base_filename,
                                        const std::string& extension,
                                        const int num_data_points,
                                        // const std::vector<double> goal_offset,
                                        const double duration_scale,
                                        const double error_threshold,
                                        const int index,
                                        const std::string base_directory,
                                        bool generate_test_data);

    static bool createDebugTrajectory(DMPType& dmp,
                                      dmp_lib::Trajectory& dmp_debug_trajectory,
                                      const dmp_lib::Trajectory& dmp_test_trajectory,
                                      std::vector<int>& debug_dimensions);

    static bool propagateDMPStepWise(DMPType& dmp,
                                     dmp_lib::Trajectory& dmp_debug_trajectory,
                                     dmp_lib::Trajectory& dmp_test_trajectory,
                                     const std::vector<int>& debug_dimensions);

  };

template<class DMPType>
  bool TestDMP<DMPType>::createDebugTrajectory(DMPType& dmp,
                                               dmp_lib::Trajectory& dmp_debug_trajectory,
                                               const dmp_lib::Trajectory& dmp_test_trajectory,
                                               std::vector<int>& debug_dimensions)
  {

      std::vector<std::string> debug_variable_names;
      debug_dimensions.clear();

      debug_variable_names.push_back("can_x");
      debug_variable_names.push_back("can_xd");
      debug_variable_names.push_back("can_time");
      debug_variable_names.push_back("can_progress");
      std::vector<std::string> dmp_variable_names = dmp.getVariableNames();

      int num_samples = dmp_test_trajectory.getNumTotalCapacity();
      double sampling_frequency = dmp_test_trajectory.getSamplingFrequency();

      const std::vector<std::pair<int, int> > indices = dmp.getIndices();
      for (int i = 0; i < (int)dmp_variable_names.size(); ++i)
      {
        int num_variables_per_dimension = debug_variable_names.size();

        debug_variable_names.push_back(dmp_variable_names[i] + "_int_x");
        debug_variable_names.push_back(dmp_variable_names[i] + "_int_xd");
        debug_variable_names.push_back(dmp_variable_names[i] + "_int_xdd");
        debug_variable_names.push_back(dmp_variable_names[i] + "_cur_x");
        debug_variable_names.push_back(dmp_variable_names[i] + "_cur_xd");
        debug_variable_names.push_back(dmp_variable_names[i] + "_cur_xdd");
        debug_variable_names.push_back(dmp_variable_names[i] + "_f");
        debug_variable_names.push_back(dmp_variable_names[i] + "_start");
        debug_variable_names.push_back(dmp_variable_names[i] + "_goal");

        int num_rfs = dmp.getTransformationSystem(indices[i].first)->getParameters(indices[i].second)->getLWRModel()->getNumRFS();
        for (int j = 0; j < num_rfs; ++j)
        {
          std::stringstream ss2;
          ss2 << j;
          debug_variable_names.push_back(dmp_variable_names[i] + "_rfs" + ss2.str());
        }
        debug_dimensions.push_back(debug_variable_names.size() - num_variables_per_dimension);
      }

      // create debug vector that will be added to the debug trajectory at each instance of time
      if (!dmp_debug_trajectory.initialize(debug_variable_names, sampling_frequency, true, num_samples))
      {
        dmp_lib::Logger::logPrintf("Could not initialize debug trajectory to store rollout.", dmp_lib::Logger::ERROR);
        return false;
      }

    return true;
  }

template<class DMPType>
  bool TestDMP<DMPType>::propagateDMPStepWise(DMPType& dmp,
                                              dmp_lib::Trajectory& dmp_debug_trajectory,
                                              dmp_lib::Trajectory& dmp_test_trajectory,
                                              const std::vector<int>& debug_dimensions)
  {

    Eigen::VectorXd debug_vector = Eigen::VectorXd::Zero(dmp_debug_trajectory.getDimension());

    // create vectors that will temporarily hold the output of the dmp
    Eigen::VectorXd desired_positions = Eigen::VectorXd::Zero(dmp.getNumDimensions());
    Eigen::VectorXd desired_velocities = Eigen::VectorXd::Zero(dmp.getNumDimensions());
    Eigen::VectorXd desired_accelerations = Eigen::VectorXd::Zero(dmp.getNumDimensions());

    const std::vector<std::pair<int, int> > indices = dmp.getIndices();

    int num_samples = dmp_test_trajectory.getNumTotalCapacity();
    double duration = num_samples / dmp_test_trajectory.getSamplingFrequency();

    bool movement_finished = false;
    while (!movement_finished)
    {
      if (!dmp.propagateStep(desired_positions, desired_velocities, desired_accelerations, movement_finished, duration, num_samples))
      {
        dmp_lib::Logger::logPrintf("Could not propagate the DMP.", dmp_lib::Logger::ERROR);
        return false;
      }

      // log debug variables
      int log_index = 0;
      debug_vector(log_index) = dmp.getCanonicalSystem()->getState()->getStateX();
      log_index++;
      debug_vector(log_index) = dmp.getCanonicalSystem()->getState()->getStateXd();
      log_index++;
      debug_vector(log_index) = dmp.getCanonicalSystem()->getState()->getTime();
      log_index++;
      debug_vector(log_index) = dmp.getProgress();
      log_index++;

      for (int i = 0; i < dmp.getNumDimensions(); ++i)
      {
        debug_vector(log_index + (i * debug_dimensions[i]) + 0)
            = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getInternalStateX();
        debug_vector(log_index + (i * debug_dimensions[i]) + 1)
            = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getInternalStateXd();
        debug_vector(log_index + (i * debug_dimensions[i]) + 2)
            = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getInternalStateXdd();
        debug_vector(log_index + (i * debug_dimensions[i]) + 3)
            = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getCurrentStateX();
        debug_vector(log_index + (i * debug_dimensions[i]) + 4)
            = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getCurrentStateXd();
        debug_vector(log_index + (i * debug_dimensions[i]) + 5)
            = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getCurrentStateXdd();
        debug_vector(log_index + (i * debug_dimensions[i]) + 6) = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getF();
        debug_vector(log_index + (i * debug_dimensions[i]) + 7) = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getStart();
        debug_vector(log_index + (i * debug_dimensions[i]) + 8) = dmp.getTransformationSystem(indices[i].first)->getStates(indices[i].second)->getGoal();

        int num_rfs = dmp.getTransformationSystem(indices[i].first)->getParameters(indices[i].second)->getLWRModel()->getNumRFS();
        for (int j = 0; j < num_rfs; ++j)
        {
          double basis_function;
          if (!dmp.getTransformationSystem(indices[i].first)->getParameters(indices[i].second)->getLWRModel()->generateBasisFunction(
                                                                                                                                     dmp.getCanonicalSystem()->getState()->getStateX(),
                                                                                                                                     j, basis_function))
          {
            dmp_lib::Logger::logPrintf("Could not get basis function of rfs >%i< of transformaton system >%i<.", dmp_lib::Logger::ERROR, j, indices[i].first);
            return false;
          }
          debug_vector(log_index + (i * debug_dimensions[i]) + 9 + j) = basis_function;
        }

      }
      if (!dmp_debug_trajectory.add(debug_vector))
      {
        dmp_lib::Logger::logPrintf("Could not add debug vector to debug trajectory.", dmp_lib::Logger::ERROR);
        return false;
      }

      if (!dmp_test_trajectory.add(desired_positions, desired_velocities, desired_accelerations))
      {
        dmp_lib::Logger::logPrintf("Could not add positions, velocities, and accelerations to trajectory.", dmp_lib::Logger::ERROR);
        return false;
      }
    }
    return true;
  }

template<class DMPType>
  bool TestDMP<DMPType>::test(DMPType& dmp,
                              const std::string& base_filename,
                              const std::string& extension,
                              const int num_data_points,
                              const std::vector<double> goal_offset,
                              const double duration_scale,
                              const double error_threshold,
                              const int index,
                              const std::string base_directory,
                              bool generate_test_data)
  {
    DMPType learning_from_trajectory_dmp = dmp;
    if (!testLearningFromTrajectory(learning_from_trajectory_dmp, base_filename, extension, /*num_data_points,*/ goal_offset, duration_scale, error_threshold,
                                    index, base_directory, generate_test_data))
    {
      dmp_lib::Logger::logPrintf("Learning from trajectory test failed.", dmp_lib::Logger::ERROR);
      return false;
    }

    DMPType learning_from_min_jerk_dmp = dmp;
    if (!testLearningFromMinJerk(learning_from_min_jerk_dmp, base_filename, extension, num_data_points, /*goal_offset,*/ duration_scale, error_threshold, index,
                                 base_directory, generate_test_data))
    {
      dmp_lib::Logger::logPrintf("Learning from minimum jerk test failed.", dmp_lib::Logger::ERROR);
      return false;
    }
    return true;
  }

template<class DMPType>
  bool TestDMP<DMPType>::testLearningFromMinJerk(DMPType& dmp,
                                                 const std::string& base_filename,
                                                 const std::string& extension,
                                                 const int num_data_points,
                                                 // const std::vector<double> goal_offset,
                                                 const double duration_scale,
                                                 const double error_threshold,
                                                 const int index,
                                                 const std::string base_directory,
                                                 bool generate_test_data)
  {

    double initial_duration = duration_scale;
    double sampling_frequency = 300.0;
    std::string data_directory_name = base_directory + "data/";
    std::string result_directory_name = base_directory + "result/";
    std::string min_jerk_fname;
    std::string fname;

    std::stringstream ss;
    ss << index;
    std::string test_id = ss.str() + "_";

    std::vector<double> initial_start;
    std::vector<double> initial_goal;
    for (int i = 0; i < dmp.getNumDimensions(); ++i)
    {
      initial_start.push_back(0.0);
      initial_goal.push_back(i);
    }

    dmp_lib::Trajectory min_jerk_trajectory;
    if (!min_jerk_trajectory.initializeWithMinJerk(dmp.getVariableNames(), sampling_frequency, initial_start, initial_goal, num_data_points, false))
    {
      dmp_lib::Logger::logPrintf("Could not create minimum jerk trajectory for learning.", dmp_lib::Logger::ERROR);
      return false;
    }

    fname.assign(result_directory_name + test_id + base_filename + extension + "_min_jerk_target" + prefix);
    if (!min_jerk_trajectory.writeToCLMCFile(fname))
    {
      dmp_lib::Logger::logPrintf("Could not write minimum jerk target trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    dmp_lib::Logger::logPrintf("Duration of minimum jerk trajectory is >%.1f< seconds. ", dmp_lib::Logger::DEBUG, min_jerk_trajectory.getDuration());
    dmp_lib::Logger::logPrintf("Minimum jerk trajectory contains >%i< samples. ", dmp_lib::Logger::DEBUG, min_jerk_trajectory.getNumContainedSamples());
    dmp_lib::Logger::logPrintf("Minimum jerk trajectory contains >%i< samples. ", dmp_lib::Logger::DEBUG, min_jerk_trajectory.getNumContainedSamples());

    // create debug trajectory to hold the debug data during learning
    dmp_lib::TrajectoryPtr learn_min_jerk_dmp_trajectory(new dmp_lib::Trajectory());

    if (!dmp.learnFromMinimumJerk(initial_start, initial_goal, sampling_frequency, initial_duration, learn_min_jerk_dmp_trajectory))
    {
      dmp_lib::Logger::logPrintf("Could not learn DMP from minimum jerk trajectory.", dmp_lib::Logger::ERROR);
      return false;
    }

    std::vector<Eigen::VectorXd> thetas;
    if (!dmp.getThetas(thetas))
    {
      dmp_lib::Logger::logPrintf("Could not get thetas from DMP.", dmp_lib::Logger::ERROR);
      return false;
    }

    fname.assign(result_directory_name + test_id + base_filename + extension + "_min_jerk_learn" + prefix);
    if (!learn_min_jerk_dmp_trajectory->writeToCLMCFile(fname, true))
    {
      dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    if (!dmp.setup())
    {
      dmp_lib::Logger::logPrintf("Could not setup DMP.", dmp_lib::Logger::ERROR);
      return false;
    }

    dmp_lib::Logger::logPrintf("Initial duration is >%.1f< seconds. ", dmp_lib::Logger::DEBUG, initial_duration);

    dmp_lib::Trajectory min_jerk_dmp_trajectory;
    dmp_lib::Logger::logPrintf("Creating minimum jerk dmp trajectory of length >%i< that will hold the rollout.", dmp_lib::Logger::DEBUG,
                               min_jerk_trajectory.getNumContainedSamples());
    if (!min_jerk_dmp_trajectory.initialize(dmp.getVariableNames(), min_jerk_trajectory.getSamplingFrequency(), false,
                                            min_jerk_trajectory.getNumContainedSamples()))
    {
      dmp_lib::Logger::logPrintf("Could not initialize trajectory to store rollout.", dmp_lib::Logger::ERROR);
      return false;
    }

    dmp_lib::Trajectory dmp_debug_trajectory;
    std::vector<int> debug_dimensions;
    if (!createDebugTrajectory(dmp, dmp_debug_trajectory, min_jerk_dmp_trajectory, debug_dimensions))
    {
      dmp_lib::Logger::logPrintf("Could not create debug trajectory.", dmp_lib::Logger::ERROR);
      return false;
    }

    if (!propagateDMPStepWise(dmp, dmp_debug_trajectory, min_jerk_dmp_trajectory, debug_dimensions))
    {
      dmp_lib::Logger::logPrintf("Could not propagate minimum jerk trajectory step wise.", dmp_lib::Logger::ERROR);
      return false;
    }

    min_jerk_fname.assign(result_directory_name + test_id + base_filename + extension + "_min_jerk" + prefix);
    if (!min_jerk_dmp_trajectory.writeToCLMCFile(min_jerk_fname))
    {
      dmp_lib::Logger::logPrintf("Could not write minimum jerk test target dmp trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    fname.assign(result_directory_name + test_id + base_filename + extension + "_min_jerk_debug" + prefix);
    if (!dmp_debug_trajectory.writeToCLMCFile(fname, true))
    {
      dmp_lib::Logger::logPrintf("Could not write minimum jerk dmp debug trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    //    if(dmp_debug_trajectory.isCompatible(*learn_min_jerk_dmp_trajectory, true))
    //    {
    //      if(!dmp_debug_trajectory.crop(10) || !learn_min_jerk_dmp_trajectory->crop(10))
    //      {
    //        return false;
    //      }
    //      if(!dmp_debug_trajectory.combine(*learn_min_jerk_dmp_trajectory))
    //      {
    //        dmp_lib::Logger::logPrintf("Could not combine trajectories.", dmp_lib::Logger::INFO);
    //        return false;
    //      }
    //      fname.assign(result_directory_name + test_id + base_filename + extension + "_min_jerk_learn_combined" + prefix);
    //      if (!dmp_debug_trajectory.writeToCLMCFile(fname, true))
    //      {
    //        dmp_lib::Logger::logPrintf("Could not write dmp trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
    //        return false;
    //      }
    //    }
    //    else
    //    {
    //      dmp_lib::Logger::logPrintf("Minimum jerk trajectories are not compatible.", dmp_lib::Logger::ERROR);
    //      return false;
    //    }


    //    dmp_lib::Trajectory min_jerk_step_combined = min_jerk_trajectory;
    //    if(!min_jerk_trajectory.combine(dmp_debug_trajectory, true))
    //    {
    //      return false;
    //    }

    std::vector<std::string> minimum_jerk_varnames_target = min_jerk_trajectory.getVariableNames();
    std::vector<std::string> new_minimum_jerk_varnames_target;
    for (int i = 0; i < (int)minimum_jerk_varnames_target.size(); ++i)
    {
      new_minimum_jerk_varnames_target.push_back("T" + minimum_jerk_varnames_target[i]);
    }
    dmp_lib::Trajectory new_min_jerk_trajectory = min_jerk_trajectory;
    if (!new_min_jerk_trajectory.setVariableNames(new_minimum_jerk_varnames_target))
    {
      dmp_lib::Logger::logPrintf("Could not set new variable names of the target minimum jerk trajectory.", dmp_lib::Logger::ERROR);
      return false;
    }

    dmp_lib::Trajectory min_jerk_combined_trajectory = new_min_jerk_trajectory;

    if (!min_jerk_combined_trajectory.combine(min_jerk_dmp_trajectory, true))
    {
      dmp_lib::Logger::logPrintf("Could not combine dmp trajectories.", dmp_lib::Logger::ERROR);
      return false;
    }

    min_jerk_fname.assign(result_directory_name + test_id + base_filename + extension + "_min_jerk_combined" + prefix);
    if (!min_jerk_combined_trajectory.writeToCLMCFile(min_jerk_fname))
    {
      dmp_lib::Logger::logPrintf("Could not write minimum jerk test target dmp trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    if (generate_test_data)
    {
      if (!min_jerk_trajectory.writeToCLMCFile(min_jerk_fname))
      {
        dmp_lib::Logger::logPrintf("Could not write minimum jerk test target dmp trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
        return false;
      }
    }
    else // do the testing...
    {
      Eigen::VectorXd min_jerk_nmse_vector = Eigen::VectorXd::Zero(min_jerk_trajectory.getDimension());

      dmp_lib::Trajectory min_jerk_target_trajectory;
      if (!min_jerk_target_trajectory.readFromCLMCFile(min_jerk_fname, dmp.getVariableNames()))
      {
        dmp_lib::Logger::logPrintf("Could not read test minimum jerk target dmp trajectory from clmc file >%s<.", dmp_lib::Logger::ERROR, min_jerk_fname.c_str());
        return false;
      }
      if (!min_jerk_dmp_trajectory.computePositionNMSE(min_jerk_target_trajectory, min_jerk_nmse_vector))
      {
        dmp_lib::Logger::logPrintf("Could not compute normalized mean squared error.", dmp_lib::Logger::ERROR);
        return false;
      }
      double total_error = min_jerk_nmse_vector.sum();
      dmp_lib::Logger::logPrintf("Normalized mean squared error of the minimum jerk trajectory is >%f<.", dmp_lib::Logger::INFO, total_error);
      if (total_error > error_threshold)
      {
        dmp_lib::Logger::logPrintf("Normalized mean squared error >%f< of the minimum jerk trajectory exceeded.", dmp_lib::Logger::ERROR, total_error);
        return false;
      }
      dmp_lib::Logger::logPrintf("Normalized mean squared error of the minimum jerk trajectory is >%f<.", dmp_lib::Logger::INFO, total_error);
    }

    return true;
  }

template<class DMPType>
  bool TestDMP<DMPType>::testLearningFromTrajectory(DMPType& dmp,
                                                    const std::string& base_filename,
                                                    const std::string& extension,
                                                    // const int num_data_points,
                                                    const std::vector<double> goal_offset,
                                                    const double duration_scale,
                                                    const double error_threshold,
                                                    const int index,
                                                    const std::string base_directory,
                                                    bool generate_test_data)
  {

    DMPType new_dmp = dmp;

    std::string data_directory_name = base_directory + "data/";
    std::string result_directory_name = base_directory + "result/";
    std::string min_jerk_fname;
    std::string fname;

    std::stringstream ss;
    ss << index;
    std::string test_id = ss.str() + "_";

    // create result directory if it does not exist
    try
    {
      boost::filesystem::create_directories(boost::filesystem::path(result_directory_name));
    }
    catch (std::exception& e)
    {
      dmp_lib::Logger::logPrintf("Could not create result directory: %s.", e.what());
      return false;
    }

    // read recorded trajectory to learn dmp
    dmp_lib::Trajectory learning_trajectory;
    fname.assign(data_directory_name + base_filename + prefix);
    if (!learning_trajectory.readFromCLMCFile(fname, dmp.getVariableNames(), false))
    {
      dmp_lib::Logger::logPrintf("Could not read clmc file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }
    if (!learning_trajectory.computeDerivatives())
    {
      dmp_lib::Logger::logPrintf("Could not compute derivatives of trajectory >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    // write to file for debugging purposes
    fname.assign(result_directory_name + test_id + base_filename + extension + "_learn" + prefix);
    if (!learning_trajectory.writeToCLMCFile(fname))
    {
      dmp_lib::Logger::logPrintf("Could not write clmc file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    // create debug trajectory to hold the debug data during learning
    dmp_lib::TrajectoryPtr learn_dmp_trajectory(new dmp_lib::Trajectory());

    // learn dmp
    if (!dmp.learnFromTrajectory(learning_trajectory, learn_dmp_trajectory))
    {
      dmp_lib::Logger::logPrintf("Could not learn from trajectory file.", dmp_lib::Logger::ERROR);
      return false;
    }

    // get initial goal and add offset
    std::vector<double> initial_goal;
    if(!dmp.getInitialGoal(initial_goal, false))
    {
      dmp_lib::Logger::logPrintf("Could not get initial goal of the DMP.", dmp_lib::Logger::ERROR);
      return false;
    }
    if(initial_goal.size() != goal_offset.size())
    {
      dmp_lib::Logger::logPrintf("Size of the DMP goal >%i< and size of the provided goal >%i< does not match.", dmp_lib::Logger::ERROR,
                                 (int)initial_goal.size(), (int)goal_offset.size());
      return false;
    }
    Eigen::VectorXd new_goal = Eigen::VectorXd::Zero(initial_goal.size());
    for (int i = 0; i < (int)initial_goal.size(); ++i)
    {
      new_goal(i) = initial_goal[i] + goal_offset[i];
    }

    // use sampling frequency from recorded trajectory
    const double sampling_frequency = learning_trajectory.getSamplingFrequency();

    // change goal of the dmp and set sampling frequency
    if (!dmp.setupSamplingFrequency(new_goal, sampling_frequency))
    {
      dmp_lib::Logger::logPrintf("Could not setup DMP.", dmp_lib::Logger::ERROR);
      return false;
    }
    // get initial duration and scale it
    double initial_duration;
    if (!dmp.getInitialDuration(initial_duration))
    {
      dmp_lib::Logger::logPrintf("Could not get initial duration of the DMP.", dmp_lib::Logger::ERROR);
      return false;
    }
    double duration = initial_duration * duration_scale;

    dmp_lib::Logger::logPrintf("Initial duration is >%f< seconds and new duration is >%f< seconds.", dmp_lib::Logger::DEBUG, initial_duration, duration);
    dmp_lib::Logger::logPrintf("Sampling frequency is >%.1f< Hz.", dmp_lib::Logger::DEBUG, sampling_frequency);

    dmp_lib::Trajectory dmp_test_trajectory;
    int num_samples = initial_duration * sampling_frequency;
    dmp_lib::Logger::logPrintf("Creating test dmp trajectory of length >%i< that will hold the rollout.", dmp_lib::Logger::DEBUG, num_samples);
    if (!dmp_test_trajectory.initialize(dmp.getVariableNames(), sampling_frequency, false, num_samples))
    {
      dmp_lib::Logger::logPrintf("Could not initialize trajectory to store rollout.", dmp_lib::Logger::ERROR);
      return false;
    }
    // create debug trajectory
    dmp_lib::Trajectory dmp_debug_trajectory;
    std::vector<int> debug_dimensions;
    if (!createDebugTrajectory(dmp, dmp_debug_trajectory, dmp_test_trajectory, debug_dimensions))
    {
      dmp_lib::Logger::logPrintf("Could not create debug trajectory.", dmp_lib::Logger::ERROR);
      return false;
    }

    // propagate the dmp stepwise
    if(!propagateDMPStepWise(dmp, dmp_debug_trajectory, dmp_test_trajectory, debug_dimensions))
    {
      dmp_lib::Logger::logPrintf("Could not propagate DMP stepwise.", dmp_lib::Logger::ERROR);
      return false;
    }

    fname.assign(result_directory_name + test_id + base_filename + extension + "_step" + prefix);
    if (!dmp_test_trajectory.writeToCLMCFile(fname))
    {
      dmp_lib::Logger::logPrintf("Could not write dmp trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
      return false;
    }

    if(dmp_debug_trajectory.isCompatible(*learn_dmp_trajectory, false))
    {
      if(!dmp_debug_trajectory.crop(10) || !learn_dmp_trajectory->crop(10))
      {
        return false;
      }
      if(!dmp_debug_trajectory.combine(*learn_dmp_trajectory))
      {
        dmp_lib::Logger::logPrintf("Could not combine trajectories.", dmp_lib::Logger::INFO);
        return false;
      }
      fname.assign(result_directory_name + test_id + base_filename + extension + "_learn_combined" + prefix);
      if (!dmp_debug_trajectory.writeToCLMCFile(fname, true))
      {
        dmp_lib::Logger::logPrintf("Could not write dmp trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
        return false;
      }
    }
    else
    {
      dmp_lib::Logger::logPrintf("Trajectories are not compatible.", dmp_lib::Logger::ERROR);
      return false;
    }


    // either evaluate or generate new test data
    fname.assign(data_directory_name + test_id + base_filename + test_target + prefix);
    min_jerk_fname.assign(data_directory_name + test_id + base_filename + test_min_jerk_target + prefix);

    if(generate_test_data)
    {
      if (!dmp_test_trajectory.writeToCLMCFile(fname))
      {
        dmp_lib::Logger::logPrintf("Could not write test target dmp trajectory to file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
        return false;
      }
    }
    else // do the testing...
    {
      dmp_lib::Trajectory target_trajectory;
      if (!target_trajectory.readFromCLMCFile(fname, dmp.getVariableNames()))
      {
        dmp_lib::Logger::logPrintf("Could not read test target dmp trajectory from clmc file >%s<.", dmp_lib::Logger::ERROR, fname.c_str());
        return false;
      }

      Eigen::VectorXd nmse_vector = Eigen::VectorXd::Zero(dmp_test_trajectory.getDimension());
      if (!dmp_test_trajectory.computePositionNMSE(target_trajectory, nmse_vector))
      {
        dmp_lib::Logger::logPrintf("Could not compute normalized mean squared error.", dmp_lib::Logger::ERROR);
        return false;
      }

      double total_error = nmse_vector.sum();
      if (total_error > error_threshold)
      {
        dmp_lib::Logger::logPrintf("Normalized mean squared error >%f< exceeded.", dmp_lib::Logger::ERROR, total_error);
        return false;
      }
      dmp_lib::Logger::logPrintf("Normalized mean squared error is >%f<.", dmp_lib::Logger::INFO, total_error);
    }
    return true;
  }

}

#endif /* TEST_DYNAMIC_MOVEMENT_PRIMITIVE_H_ */
