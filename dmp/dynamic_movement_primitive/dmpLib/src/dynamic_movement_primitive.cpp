/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		dynamic_movement_primitive.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 3, 2010

 *********************************************************************/

// system include
#include <vector>
#include <string>
#include <stdio.h>

// local include
#include <dmp_lib/dynamic_movement_primitive.h>
#include <dmp_lib/logger.h>
#include <dmp_lib/utilities.h>

using namespace std;
using namespace boost;
using namespace Eigen;

namespace dmp_lib
{

/*! Minimum number of data points
 */
static const int MIN_NUM_DATA_POINTS = 70;

bool DynamicMovementPrimitive::initialize(DMPParamPtr parameters,
                                          DMPStatePtr state,
                                          std::vector<TSPtr>& transformation_systems,
                                          CSPtr canonical_system)
{
  Logger::logPrintf("Initializing DMP.", Logger::DEBUG);
  parameters_ = parameters;
  state_ = state;
  canonical_system_= canonical_system;
  transformation_systems_ = transformation_systems;
  if(!setupIndices())
  {
    return (initialized_ = false);
  }
  zero_feedback_ = Eigen::VectorXd::Zero(indices_.size());
  return (initialized_ = true);
}

bool DynamicMovementPrimitive::initialize(DMPParamPtr parameters,
                                          vector<TSPtr>& transformation_systems,
                                          CSPtr canonical_system)
{
  DMPStatePtr state(new DynamicMovementPrimitiveState());
  return initialize(parameters, state, transformation_systems, canonical_system);
}

bool DynamicMovementPrimitive::setupIndices()
{
  indices_.clear();
  for (int i = 0; i < static_cast<int> (transformation_systems_.size()); ++i)
  {
    if(!transformation_systems_[i]->isInitialized())
    {
      Logger::logPrintf("Cannot initialize DMP from uninitialized transformation system.", Logger::ERROR);
      return false;
    }
    transformation_systems_[i]->reset();
    Logger::logPrintf("Initializing DMP transformation system with >%i< dimensions.", Logger::DEBUG, transformation_systems_[i]->getNumDimensions());
    for (int j = 0; j < transformation_systems_[i]->getNumDimensions(); ++j)
    {
      pair<int, int> index_pair;
      index_pair.first = i;
      index_pair.second = j;
      indices_.push_back(index_pair);
    }
  }
  return true;
}

bool DynamicMovementPrimitive::isCompatible(const DynamicMovementPrimitive& other_dmp) const
{
  if (!initialized_)
  {
    Logger::logPrintf("DMP not uninitialized DMP, not compatible.", Logger::ERROR);
    return false;
  }
  if (!other_dmp.isInitialized())
  {
    Logger::logPrintf("Other DMP not uninitialized DMP, not compatible.", Logger::ERROR);
    return false;
  }
  return (parameters_->isCompatible(*other_dmp.parameters_) && state_->isCompatible(*other_dmp.state_));
}

void DynamicMovementPrimitive::changeType(const int type)
{
  assert(initialized_);
  parameters_->type_ = type;
}

bool DynamicMovementPrimitive::hasType(const int type) const
{
  assert(initialized_);
  return (type == parameters_->type_);
}

int DynamicMovementPrimitive::getType() const
{
  assert(initialized_);
  return parameters_->type_;
}

int DynamicMovementPrimitive::getSeq() const
{
  assert(initialized_);
  return state_->seq_;
}

void DynamicMovementPrimitive::setId(const int id)
{
  assert(initialized_);
  parameters_->id_ = id;
}

int DynamicMovementPrimitive::getId() const
{
  assert(initialized_);
  return parameters_->id_;
}

bool DynamicMovementPrimitive::get(DMPParamConstPtr& parameters,
                                   DMPStateConstPtr& state) const
{
  if (!initialized_)
  {
    Logger::logPrintf("DMP is not initialized, cannot return parameters and state.", Logger::ERROR);
    return false;
  }
  parameters = parameters_;
  state = state_;
  return true;
}

bool DynamicMovementPrimitive::prepareTrajectory(Trajectory& trajectory)
{

  if (!trajectory.isInitialized())
  {
    Logger::logPrintf("Trajectory is not initialized. Cannot prepare trajectory.", Logger::ERROR);
    return false;
  }

  if (trajectory.getNumContainedSamples() < MIN_NUM_DATA_POINTS)
  {
    Logger::logPrintf("Trajectory has >%i< samples, but must have at least >%i<. Cannot prepare trajectory.",
                      Logger::ERROR, trajectory.getNumContainedSamples(), MIN_NUM_DATA_POINTS);
    return false;
  }

  if (trajectory.getSamplingFrequency() <= 0)
  {
    Logger::logPrintf("Invalid sampling frequency >%f<. Cannot prepare trajectory.", Logger::ERROR, trajectory.getSamplingFrequency());
    return false;
  }

  vector<string> variable_names = getVariableNames();
  if (!trajectory.rearange(variable_names))
  {
    string all_variable_names = "";
    for (int i = 0; i < (int)variable_names.size(); ++i)
    {
      all_variable_names.append(variable_names[i]);
      if (i + 1 < (int)variable_names.size())
      {
        all_variable_names.append(" ");
      }
    }
    Logger::logPrintf("Could not rearange data traces with variable names >%s< of the trajectory.", Logger::ERROR, all_variable_names.c_str());
    return false;
  }

  return true;
}

bool DynamicMovementPrimitive::learnFromThetas(const std::vector<Eigen::VectorXd>& thetas,
                                               const Eigen::VectorXd& initial_start,
                                               const Eigen::VectorXd& initial_goal,
                                               const double sampling_frequency,
                                               const double initial_duration)
{
  assert(initialized_);
  if((int)thetas.size() != getNumDimensions())
  {
    Logger::logPrintf("There are >%i< parameter vectors, but the DMP has >%i< dimensions.",
                      Logger::ERROR, thetas.size(), getNumDimensions());
    return (state_->is_learned_ = false);
  }
  if(initial_start.size() != getNumDimensions() || initial_goal.size() != getNumDimensions())
  {
    Logger::logPrintf("Size of provided initial start >%i< or initial goal >%i< does not match number of dimensions of the DMP.",
                      Logger::ERROR, initial_start.size(), initial_goal.size(), getNumDimensions());
    return (state_->is_learned_ = false);
  }

  // set y0 to start state of trajectory and set goal to end of the trajectory
  for (int i = 0; i < getNumDimensions(); i++)
  {
    // set initial start and initial goal
    if(!transformation_systems_[indices_[i].first]->setInitialStart(indices_[i].second, initial_start(i)))
    {
      Logger::logPrintf("Could not set initial start when setting the theta vector.", Logger::ERROR);
      return (state_->is_learned_ = false);
    }
    if (!transformation_systems_[indices_[i].first]->setInitialGoal(indices_[i].second, initial_goal(i)))
    {
      Logger::logPrintf("Could not set initial start when setting the theta vector.", Logger::ERROR);
      return (state_->is_learned_ = false);
    }
  }

  // set teaching duration to the duration of the trajectory
  parameters_->teaching_duration_ = initial_duration;

  assert(state_->current_time_.setDeltaT(static_cast<double> (1.0) / static_cast<double> (sampling_frequency)));
  assert(state_->current_time_.setTau(parameters_->teaching_duration_));

  parameters_->initial_time_ = state_->current_time_;

  // compute alpha_x such that the canonical system drops
  // below the cutoff when the trajectory has finished
  if (!canonical_system_->parameters_->setCutoff(parameters_->cutoff_))
  {
    Logger::logPrintf("Could not set cutoff of the canonical system. Cannot learn DMP from thetas.", Logger::ERROR);
    return (state_->is_learned_ = false);
  }

  // reset training samples counter
  state_->num_training_samples_ = 0;

  if (!setThetas(thetas))
  {
    Logger::logPrintf("Could not set theta parameters. Cannot learn DMP from thetas.", Logger::ERROR);
    return (state_->is_learned_ = false);
  }

  Logger::logPrintf("Done learning DMP from thetas.", Logger::INFO);
  return (state_->is_learned_ = true);
}

bool DynamicMovementPrimitive::learnFromThetas(const std::vector<Eigen::VectorXd>& thetas,
                                               const std::vector<double>& initial_start,
                                               const std::vector<double>& initial_goal,
                                               const double sampling_frequency,
                                               const double initial_duration)
{
  return learnFromThetas(thetas,
                         VectorXd::Map(&initial_start[0], initial_start.size()),
                         VectorXd::Map(&initial_goal[0], initial_goal.size()),
                         sampling_frequency,
                         initial_duration);
}

bool DynamicMovementPrimitive::learnFromTrajectory(const Trajectory& demo_trajectory, TrajectoryPtr debug_trajectory)
{
  assert(initialized_);
  assert(demo_trajectory.isInitialized());
  Logger::logPrintf("Learning >%i< dimensional >%s< DMP from trajectory with >%i< dimensions.", Logger::INFO,
                    getNumDimensions(), getVersionString().c_str(), demo_trajectory.getDimension());

  Trajectory trajectory = demo_trajectory;
  if (!prepareTrajectory(trajectory))
  {
    return (state_->is_learned_ = false);
  }

  if (debug_trajectory)
  {
    if (!createDebugTrajectory(*debug_trajectory, demo_trajectory))
    {
      Logger::logPrintf("Could not create debug trajectory.", Logger::ERROR);
      return false;
    }
  }

  // set teaching duration to the duration of the trajectory
  parameters_->teaching_duration_ = static_cast<double> (trajectory.getNumContainedSamples()) / static_cast<double> (trajectory.getSamplingFrequency());

  assert(state_->current_time_.setDeltaT(static_cast<double> (1.0) / static_cast<double> (trajectory.getSamplingFrequency())));
  assert(state_->current_time_.setTau(parameters_->teaching_duration_));

  parameters_->initial_time_ = state_->current_time_;

  // compute alpha_x such that the canonical system drops
  // below the cutoff when the trajectory has finished
  if (!canonical_system_->parameters_->setCutoff(parameters_->cutoff_))
  {
    Logger::logPrintf("Could not set cutoff of the canonical system. Cannot learn DMP from trajectory.", Logger::ERROR);
    return (state_->is_learned_ = false);
  }

  // reset canonical system
  canonical_system_->reset();

  // reset training samples counter
  state_->num_training_samples_ = 0;

  // obtain start and goal position
  VectorXd start = VectorXd::Zero(getNumDimensions());
  if (!trajectory.getStartPosition(start))
  {
    Logger::logPrintf("Could not get the start position of the trajectory. Cannot learn DMP from trajectory.", Logger::ERROR);
    return (state_->is_learned_ = false);
  }
  VectorXd goal = VectorXd::Zero(getNumDimensions());
  if (!trajectory.getEndPosition(goal))
  {
    Logger::logPrintf("Could not get the goal position of the trajectory. Cannot learn DMP from trajectory.", Logger::ERROR);
    return (state_->is_learned_ = false);
  }

  // set y0 to start state of trajectory and set goal to end of the trajectory
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    // set internal state (especially velocity and acceleration) to zero
    transformation_systems_[indices_[i].first]->reset();

    // set start and goal
    if(!transformation_systems_[indices_[i].first]->setStart(indices_[i].second, start(i)))
    {
      return false;
    }
    if(!transformation_systems_[indices_[i].first]->setGoal(indices_[i].second, goal(i)))
    {
      return false;
    }

    // set current state to start state (zero out velocities and accelerations) //TODO: change this...
    if(!transformation_systems_[indices_[i].first]->setCurrentState(indices_[i].second, State(start(i), 0.0, 0.0)))
    {
      return false;
    }

    // set initial start and goal
    if(!transformation_systems_[indices_[i].first]->setInitialStart(indices_[i].second, start(i)))
    {
      return false;
    }
    if(!transformation_systems_[indices_[i].first]->setInitialGoal(indices_[i].second, goal(i)))
    {
      return false;
    }
  }

  vector<vector<State> > target_states;
  target_states.resize(getNumTransformationSystems());
  for (int i = 0; i < getNumTransformationSystems(); ++i)
  {
    target_states[i].resize(transformation_systems_[i]->getNumDimensions());
  }

  for (int row_index = 0; row_index < trajectory.getNumContainedSamples(); ++row_index)
  {
    double t = 0, td = 0, tdd = 0;
    int trajectory_index = 0;
    for (int i = 0; i < getNumTransformationSystems(); ++i)
    {
      // get state
      for (int j = 0; j < transformation_systems_[i]->getNumDimensions(); ++j)
      {
        assert(trajectory.getTrajectoryPosition(row_index, trajectory_index, t));
        assert(trajectory.getTrajectoryVelocity(row_index, trajectory_index, td));
        assert(trajectory.getTrajectoryAcceleration(row_index, trajectory_index, tdd));
        target_states[i][j].set(t, td, tdd);
        trajectory_index++;
      }

      // fit state
      if (!transformation_systems_[i]->integrateAndFit(target_states[i], canonical_system_->state_, state_->current_time_))
      {
        Logger::logPrintf("Could not integrate and fit transformation system >%i<. Cannot learn DMP from trajectory.", Logger::ERROR, i);
        return (state_->is_learned_ = false);
      }
    }

    state_->num_training_samples_++;
    canonical_system_->integrate(state_->current_time_);

    if (debug_trajectory)
    {
      if (!logDebugTrajectory(*debug_trajectory))
      {
        return false;
      }
    }
  }

  if (!learnTransformationTarget())
  {
    Logger::logPrintf("Could not learn transformation target. Cannot learn DMP from trajectory.", Logger::ERROR);
    return (state_->is_learned_ = false);
  }

  // TODO: remove this
  // vector<string> tmp_names = getVariableNames();
  // string tmp_name = "/tmp/learn_demo_" + tmp_names[0] + ".clmc";
  // assert(demo_trajectory.writeToCLMCFile(tmp_name));

  if (debug_trajectory)
  {
    assert(debug_trajectory->writeToCLMCFile("/tmp/learn_debug.clmc", true));
  }

  Logger::logPrintf("Done learning DMP from trajectory.", Logger::INFO);
  return (state_->is_learned_ = true);
}

bool DynamicMovementPrimitive::learnFromMinimumJerk(const Eigen::VectorXd& start,
                                                    const Eigen::VectorXd& goal,
                                                    const double sampling_frequency,
                                                    const double initial_duration,
                                                    TrajectoryPtr debug_trajectory)
{
  assert(initialized_);
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(goal);
  std::vector<double> initial_durations;
  initial_durations.push_back(initial_duration);
  return learnFromMinimumJerk(waypoints, sampling_frequency, initial_durations, debug_trajectory);
}

bool DynamicMovementPrimitive::learnFromMinimumJerk(const std::vector<double>& start,
                                                    const std::vector<double>& goal,
                                                    const double sampling_frequency,
                                                    const double initial_duration,
                                                    TrajectoryPtr debug_trajectory)
{
  return learnFromMinimumJerk(VectorXd::Map(&start[0], start.size()),
                              VectorXd::Map(&goal[0], goal.size()),
                              sampling_frequency,
                              initial_duration,
                              debug_trajectory);
}

bool DynamicMovementPrimitive::learnFromMinimumJerk(const std::vector<Eigen::VectorXd>& waypoints,
                                                    const double sampling_frequency,
                                                    const std::vector<double>& initial_durations,
                                                    TrajectoryPtr debug_trajectory)
{
  if (waypoints.size() != initial_durations.size() + 1)
  {
    Logger::logPrintf("There are >%i< initial durations for >%i< waypoints. Cannot learn DMP minimum jerk trajectory.", Logger::ERROR,
                      initial_durations.size()+1, waypoints.size());
    return false;
  }

  std::vector<int> num_samples;
  int num_total_samples = 0;
  double total_initial_duration = 0.0;
  for (int i = 0; i < (int)initial_durations.size(); ++i)
  {
    total_initial_duration += initial_durations[i];
    int samples = initial_durations[i] * sampling_frequency;
    num_total_samples += samples;
    num_samples.push_back(samples);
  }

  Logger::logPrintf("Learning DMP from minimum jerk trajectory with >%i< waypoints, >%i< samples, and sampled at >%.1f< Hz.",
                    Logger::DEBUG, (int)waypoints.size(), num_total_samples, sampling_frequency);

  Trajectory min_jerk_trajectory;
  if (!min_jerk_trajectory.initializeWithMinJerk(getVariableNames(), sampling_frequency, waypoints, num_samples, false))
  {
    Logger::logPrintf("Could not create minimum jerk trajectory for learning.", Logger::ERROR);
    return false;
  }
  // assert(min_jerk_trajectory.writeToCLMCFile("/tmp/debug_min_jerk.clmc"));
  return learnFromTrajectory(min_jerk_trajectory, debug_trajectory);
}

bool DynamicMovementPrimitive::learnFromMinimumJerk(const std::vector<std::vector<double> >& waypoints,
                                                    const double sampling_frequency,
                                                    const std::vector<double>& initial_durations,
                                                    TrajectoryPtr debug_trajectory)
{
  std::vector<VectorXd> eigen_waypoints;
  for(int i=0; i<(int)waypoints.size(); ++i)
  {
    VectorXd waypoint = VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return learnFromMinimumJerk(eigen_waypoints, sampling_frequency, initial_durations, debug_trajectory);
}


bool DynamicMovementPrimitive::createDebugTrajectory(Trajectory& debug_trajectory, const Trajectory& trajectory)
{
  Logger::logPrintf("Creating debug trajectory.", Logger::INFO);
  std::vector<std::string> variable_names;
  variable_names.push_back("Tcan_x");
  variable_names.push_back("Tcan_xd");
  variable_names.push_back("Tcan_time");
  variable_names.push_back("Tcan_progress");
  debug_dimensions_.clear();
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    int num_variables_per_dimension = variable_names.size();
    std::string name;
    if(!transformation_systems_[indices_[i].first]->getName(indices_[i].second, name))
    {
      return false;
    }
    variable_names.push_back("T" + name + "_tar_x");
    variable_names.push_back("T" + name + "_tar_xd");
    variable_names.push_back("T" + name + "_tar_xdd");
    variable_names.push_back("T" + name + "_int_x");
    variable_names.push_back("T" + name + "_int_xd");
    variable_names.push_back("T" + name + "_int_xdd");
    variable_names.push_back("T" + name + "_cur_x");
    variable_names.push_back("T" + name + "_cur_xd");
    variable_names.push_back("T" + name + "_cur_xdd");
    variable_names.push_back("T" + name + "_ft");
    variable_names.push_back("T" + name + "_start");
    variable_names.push_back("T" + name + "_goal");

    int num_rfs = transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getNumRFS();
    for(int j=0; j<num_rfs; ++j)
    {
      stringstream ss;
      ss << j;
      variable_names.push_back("T" + name + "_rfs" + ss.str());
    }
    debug_dimensions_.push_back(variable_names.size() - num_variables_per_dimension);
  }
  debug_trajectory.initialize(variable_names, trajectory.getSamplingFrequency(), true, trajectory.getNumContainedSamples());
  return true;
}

bool DynamicMovementPrimitive::logDebugTrajectory(Trajectory& debug_trajectory)
{
  VectorXd debug_vector = VectorXd::Zero(debug_trajectory.getDimension());
  int index = 0;
  debug_vector(index) = canonical_system_->getState()->getStateX();
  index++;
  debug_vector(index) = canonical_system_->getState()->getStateXd();
  index++;
  debug_vector(index) = canonical_system_->getState()->getTime();
  index++;
  debug_vector(index) = getProgress();
  index++;
  // int num_vars_per_dim = debug_trajectory.getDimension();
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    debug_vector(index + (i*debug_dimensions_[i]) +0) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getTargetStateX();
    debug_vector(index + (i*debug_dimensions_[i]) +1) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getTargetStateXd();
    debug_vector(index + (i*debug_dimensions_[i]) +2) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getTargetStateXdd();
    debug_vector(index + (i*debug_dimensions_[i]) +3) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getInternalStateX();
    debug_vector(index + (i*debug_dimensions_[i]) +4) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getInternalStateXd();
    debug_vector(index + (i*debug_dimensions_[i]) +5) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getInternalStateXdd();
    debug_vector(index + (i*debug_dimensions_[i]) +6) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getCurrentStateX();
    debug_vector(index + (i*debug_dimensions_[i]) +7) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getCurrentStateXd();
    debug_vector(index + (i*debug_dimensions_[i]) +8) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getCurrentStateXdd();
    debug_vector(index + (i*debug_dimensions_[i]) +9) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getFT();
    debug_vector(index + (i*debug_dimensions_[i]) +10) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getStart();
    debug_vector(index + (i*debug_dimensions_[i]) +11) = transformation_systems_[indices_[i].first]->states_[indices_[i].second]->getGoal();

    int num_rfs = transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getNumRFS();
    for(int j=0; j<num_rfs; ++j)
    {
      double basis_function;
      if(!transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->generateBasisFunction(canonical_system_->getState()->getStateX(), j, basis_function))
      {
        Logger::logPrintf("Could not get basis function of rfs >%i< of transformaton system >%i<.", Logger::ERROR, j, indices_[i].first);
        return false;
      }
      debug_vector(index + (i*debug_dimensions_[i]) +12 +j) = basis_function;
    }
  }
  return debug_trajectory.add(debug_vector);
}

bool DynamicMovementPrimitive::learnTransformationTarget()
{
  assert(initialized_);
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    // ignore the first dimension of the quaternion transformation system
    if (!(transformation_systems_[indices_[i].first]->integration_method_ == TransformationSystem::QUATERNION
        && indices_[i].second == 0))
    {
      if (transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_input_.empty())
      {
        Logger::logPrintf("Transformaion system >%i< dimension >%i< has no function input.", Logger::ERROR, indices_[i].first, indices_[i].second);
        return false;
      }
      if (transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_target_.empty())
      {
        Logger::logPrintf("Transformaion system >%i< dimension >%i< has no function target.", Logger::ERROR, indices_[i].first, indices_[i].second);
        return false;
      }
      if (transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_input_.size()
          != transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_target_.size())
      {
        Logger::logPrintf("Transformaion system >%i< dimension >%i< has incompatible function sizes (input >%i< vs. target = >%i<).", Logger::ERROR,
                          indices_[i].first, indices_[i].second,
                          (int)transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_input_.size(),
                          (int)transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_target_.size());
        return false;
      }

      Eigen::Map<VectorXd> input = VectorXd::Map(&transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_input_[0],
                                                 transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_input_.size());
      Eigen::Map<VectorXd> target = VectorXd::Map(&transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_target_[0],
                                                  transformation_systems_[indices_[i].first]->states_[indices_[i].second]->function_target_.size());

      if (!transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->learn(input, target))
      {
        Logger::logPrintf("Could not learn weights of transformation system >%i<.", Logger::ERROR, i);
        return false;
      }
    }
  }
  return true;
}

bool DynamicMovementPrimitive::setup(const VectorXd& start,
                                     const VectorXd& goal,
                                     const double movement_duration,
                                     const double sampling_frequency)
{
  assert(initialized_);
  if (!state_->is_learned_)
  {
    Logger::logPrintf("DMP has not been trained yet.", Logger::ERROR);
    return (state_->is_setup_ = false);
  }

  assert(start.size() == getNumDimensions());
  assert(goal.size() == getNumDimensions());

  if (movement_duration <= 0)
  {
    Logger::logPrintf("Movement duration >%f< is invalid.", Logger::ERROR, movement_duration);
    return (state_->is_setup_ = false);
  }

  if (sampling_frequency <= 0)
  {
    Logger::logPrintf("Sampling frequency >%f< [Hz] is invalid.", Logger::ERROR, sampling_frequency);
    return (state_->is_setup_ = false);
  }

  // reset canonical system
  canonical_system_->reset();
  if (!canonical_system_->parameters_->setCutoff(parameters_->cutoff_))
  {
    Logger::logPrintf("Could not set cutoff of the canonical system.", Logger::ERROR);
    return (state_->is_setup_ = false);
  }

  assert(state_->current_time_.setTau(movement_duration));
  assert(state_->current_time_.setDeltaT(static_cast<double> (1.0) / static_cast<double> (sampling_frequency)));

  for (int i = 0; i < getNumDimensions(); ++i)
  {
    // set internal variables to zero
    transformation_systems_[indices_[i].first]->reset();

    // set start and goal
    if(!transformation_systems_[indices_[i].first]->setStart(indices_[i].second, start(i)))
    {
      return false;
    }
    if(!transformation_systems_[indices_[i].first]->setGoal(indices_[i].second, goal(i)))
    {
      return false;
    }

    // set current state to start state (position and velocity)
    if(!transformation_systems_[indices_[i].first]->setCurrentState(indices_[i].second, State(start(i), 0.0, 0.0)))
    {
      return false;
    }
  }

  // reset the generated samples counter
  state_->num_generated_samples_ = 0;

  // start is set
  state_->is_start_set_ = true;
  return (state_->is_setup_ = true);
}

bool DynamicMovementPrimitive::setupDuration(const VectorXd& start,
                                             const VectorXd& goal,
                                             const double movement_duration)
{
  assert(initialized_);
  double initial_sampling_frequency = 0;
  assert(getInitialSamplingFrequency(initial_sampling_frequency));
  return setup(start, goal, movement_duration, initial_sampling_frequency);
}

bool DynamicMovementPrimitive::setupSamplingFrequency(const VectorXd& start,
                                                      const VectorXd& goal,
                                                      const double sampling_frequency)
{
  assert(initialized_);
  return setup(start, goal, parameters_->initial_time_.getTau(), sampling_frequency);
}

bool DynamicMovementPrimitive::setup(const VectorXd &goal,
                                     const double movement_duration,
                                     const double sampling_frequency)
{
  assert(initialized_);
  VectorXd start = VectorXd(getNumDimensions());
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getInitialStart(indices_[i].second, start(i)))
    {
      return false;
    }
  }
  return setup(start, goal, movement_duration, sampling_frequency);
}

bool DynamicMovementPrimitive::setupDuration(const Eigen::VectorXd& goal,
                                             const double movement_duration)
{
  assert(initialized_);
  // setup dmp timings to the timings used during learning
  double initial_sampling_frequency = 0;
  assert(getInitialSamplingFrequency(initial_sampling_frequency));
  bool result = setup(goal, movement_duration, initial_sampling_frequency);
  // TODO: check whether this is neccessary
  state_->current_time_ = parameters_->initial_time_;
  return result;
}

bool DynamicMovementPrimitive::setupSamplingFrequency(const Eigen::VectorXd& goal,
                                                      const double sampling_frequency)
{
  assert(initialized_);
  return setup(goal, parameters_->initial_time_.getTau(), sampling_frequency);
}

bool DynamicMovementPrimitive::setupSamplingFrequency(const double sampling_frequency)
{
  assert(initialized_);
  VectorXd goal = VectorXd::Zero(getNumDimensions());
  for (int i = 0; i < getNumDimensions(); i++)
  {
    if(!transformation_systems_[indices_[i].first]->getInitialGoal(indices_[i].second, goal(i)))
    {
      return false;
    }
  }
  return setup(goal, parameters_->initial_time_.getTau(), sampling_frequency);
}

bool DynamicMovementPrimitive::setupDuration(const double movement_duration)
{
  assert(initialized_);
  VectorXd goal = VectorXd::Zero(getNumDimensions());
  for (int i = 0; i < getNumDimensions(); i++)
  {
    if(!transformation_systems_[indices_[i].first]->getInitialGoal(indices_[i].second, goal(i)))
    {
      return false;
    }
  }
  // setup dmp timings to the timings used during learning
  double initial_sampling_frequency = 0;
  assert(getInitialSamplingFrequency(initial_sampling_frequency));
  return setup(goal, movement_duration, initial_sampling_frequency);
}

bool DynamicMovementPrimitive::setup(const VectorXd& goal)
{
  assert(initialized_);
  // setup dmp timings to the timings used during learning
  double initial_sampling_frequency = 0;
  assert(getInitialSamplingFrequency(initial_sampling_frequency));
  return setup(goal, parameters_->initial_time_.getTau(), initial_sampling_frequency);
}

bool DynamicMovementPrimitive::setup()
{
  assert(initialized_);
  VectorXd goal = VectorXd::Zero(getNumDimensions());
  for (int i = 0; i < getNumDimensions(); i++)
  {
    if(!transformation_systems_[indices_[i].first]->getInitialGoal(indices_[i].second, goal(i)))
    {
      return false;
    }
  }
  return setup(goal);
}

vector<string> DynamicMovementPrimitive::getVariableNames() const
{
  assert(initialized_);
  vector < string > variable_names;
  for (vector<TSPtr>::const_iterator ti = transformation_systems_.begin(); ti != transformation_systems_.end(); ++ti)
  {
    std::vector<std::string> names;
    (*ti)->getNames(names);
    variable_names.insert(variable_names.end(), names.begin(), names.end());
  }
  return variable_names;
}

bool DynamicMovementPrimitive::getGoal(const std::vector<std::string> variable_names,
                                       std::vector<double> &goal) const
{
  assert(initialized_);
  goal.clear();
  goal.resize(variable_names.size());
  if(variable_names.empty())
  {
    Logger::logPrintf("Cannot get goal from DMP without specifying variable names.", Logger::ERROR);
    return false;
  }

  int index = 0;
  for (std::vector<std::string>::const_iterator vi = variable_names.begin(); vi != variable_names.end(); ++vi)
  {
    bool found = false;
    for (int i = 0; i < getNumDimensions(); ++i)
    {
      std::string name;
      if(!transformation_systems_[indices_[i].first]->getName(indices_[i].second, name))
      {
        Logger::logPrintf("Cannot retreive variable name from transformation system >%i<. Cannot get goal.", Logger::ERROR, i);
        return false;
      }

      if (vi->compare(name) == 0)
      {
        found = true;
        if(!transformation_systems_[indices_[i].first]->getGoal(indices_[i].second, goal[index]))
        {
          return false;
        }
      }
    }
    if (!found)
    {
      Logger::logPrintf("Could not find requested transformation system name >%s<.", Logger::ERROR, vi->c_str());
      return false;
    }
    index++;
  }
  return true;
}

bool DynamicMovementPrimitive::getStart(const std::vector<std::string> variable_names,
                                        std::vector<double> &start) const
{
  assert(initialized_);
  start.clear();
  start.resize(variable_names.size());
  if(variable_names.empty())
  {
    Logger::logPrintf("Cannot get start from DMP without specifying variable names.", Logger::ERROR);
    return false;
  }

  int index = 0;
  for (std::vector<std::string>::const_iterator vi = variable_names.begin(); vi != variable_names.end(); ++vi)
  {
    bool found = false;
    for (int i = 0; i < getNumDimensions(); ++i)
    {
      std::string name;
      if(!transformation_systems_[indices_[i].first]->getName(indices_[i].second, name))
      {
        Logger::logPrintf("Cannot retreive variable name from transformation system >%i<. Cannot get start.", Logger::ERROR, i);
        return false;
      }

      if (vi->compare(name) == 0)
      {
        found = true;
        if(!transformation_systems_[indices_[i].first]->getStart(indices_[i].second, start[index]))
        {
          return false;
        }
      }
    }
    if (!found)
    {
      Logger::logPrintf("Could not find requested transformation system name >%s<.", Logger::ERROR, vi->c_str());
      return false;
    }
    index++;
  }
  return true;
}

bool DynamicMovementPrimitive::isReadyToPropagate()
{
  if (!state_->is_learned_)
  {
    Logger::logPrintf("DMP is not learned.", Logger::ERROR);
    return false;
  }
  if ((canonical_system_->getState()->getCanX() > (parameters_->cutoff_/2.0)) && !state_->is_setup_)
  {
    Logger::logPrintf("DMP is not setup. Need to be setup first using on of the setup() functions.", Logger::ERROR);
    return false;
  }
  if ((canonical_system_->getState()->getCanX() > (parameters_->cutoff_/2.0)) && !state_->is_start_set_)
  {
    Logger::logPrintf("Start of the dmp is not set.", Logger::ERROR);
    return false;
  }
  return true;
}

bool DynamicMovementPrimitive::propagateFull(Trajectory& trajectory,
                                             const double sampling_duration,
                                             const int num_samples)
{
  assert(initialized_);
  if(!isReadyToPropagate() || (sampling_duration < 1e-10) || num_samples < 1)
  {
    return false;
  }

  // initialize trajectory
  double special_sampling_frequency = static_cast<double> (num_samples) / (sampling_duration);
  if (!trajectory.initialize(getVariableNames(), special_sampling_frequency, false, num_samples))
  {
    Logger::logPrintf("Could not initialize trajectory to store rollout.", Logger::ERROR);
    return false;
  }

  VectorXd desired_positions = VectorXd::Zero(getNumDimensions());
  VectorXd desired_velocities = VectorXd::Zero(getNumDimensions());
  VectorXd desired_accelerations = VectorXd::Zero(getNumDimensions());
  bool movement_finished = false;
  while (!movement_finished)
  {
    if (!propagateStep(desired_positions, desired_velocities, desired_accelerations, movement_finished, sampling_duration, num_samples))
    {
      Logger::logPrintf("Could not propagate dmp.", Logger::ERROR);
      return false;
    }

    if (!trajectory.add(desired_positions, desired_velocities, desired_accelerations))
    {
      Logger::logPrintf("Could not add positions, velocities, and accelerations to trajectory.", Logger::ERROR);
      return false;
    }
  }
  return true;
}

bool DynamicMovementPrimitive::propagateFull(Trajectory& trajectory,
                                             const double sampling_duration)
{
  assert(initialized_);
  double initial_sampling_frequency = 0.0;
  if(!getInitialSamplingFrequency(initial_sampling_frequency))
  {
    Logger::logPrintf("Cannot get initial sampling frequency. Cannot propagate DMP full.", Logger::ERROR);
    return false;
  }
  const int num_samples = static_cast<int> (floor(sampling_duration * initial_sampling_frequency));
  return propagateFull(trajectory, sampling_duration, num_samples);
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::integrate(const int num_iteration, const VectorXd& feedback)
{
  assert(initialized_);
  int index = 0;
  for (int i = 0; i < getNumTransformationSystems(); ++i)
  {
    int num_dimesions = transformation_systems_[i]->getNumDimensions();
    if (!transformation_systems_[i]->integrate(canonical_system_->state_, state_->current_time_, feedback.segment(index, num_dimesions), num_iteration))
    {
      return false;
    }
    index += num_dimesions;
  }
  return true;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::propagateStep(VectorXd& desired_positions,
                                             VectorXd& desired_velocities,
                                             VectorXd& desired_accelerations,
                                             bool& movement_finished,
                                             const Eigen::VectorXd& feedback,
                                             const double sampling_duration,
                                             const int num_samples)
{
  assert(initialized_);
  movement_finished = false;

  if ((desired_positions.size() != desired_velocities.size())
      || (desired_positions.size() != desired_accelerations.size())
      || (desired_positions.size() < getNumDimensions()))
  {
    Logger::logPrintf("Number of desired positions >%i<, velocities >%i<, or accelerations >%i< is incorrect, it should be >%i<. (Real-time violation).",
                      Logger::ERROR, desired_positions.size(), desired_velocities.size(), desired_accelerations.size(), getNumDimensions());
    movement_finished = true;
    return false;
  }
  if (num_samples <= 0)
  {
    Logger::logPrintf("Number of samples >%i< is invalid. (Real-time violation).", Logger::ERROR, num_samples);
    movement_finished = true;
    return false;
  }
  if (!isReadyToPropagate())
  {
    movement_finished = true;
    return false;
  }
  if (!state_->current_time_.setDeltaT(sampling_duration / static_cast<double> (num_samples)))
  {
    movement_finished = true;
    return false;
  }
  if (!state_->current_time_.setTau(sampling_duration))
  {
    movement_finished = true;
    return false;
  }
  if (feedback.size() < getNumDimensions())
  {
    Logger::logPrintf("Size of feedback vector >%i< does not match number of dimension >%i<. (Real-time violation).", Logger::ERROR,
                      feedback.size(), getNumDimensions());
    movement_finished = true;
    return false;
  }

  // TODO: think about this...
  // double dt_total = sampling_duration / static_cast<double> (num_samples);
  // double dt_threshold = static_cast<double> (1.0) / DEFAULT_SAMPLING_FREQUENCY;
  // int num_iteration = ceil(dt_total / dt_threshold);

  int num_iteration = 1;

  // integrate the system, make sure that all internal variables are set properly
  if (!integrate(num_iteration, feedback))
  {
    Logger::logPrintf("Problem while integrating the transformation system. (Real-time violation).", Logger::ERROR);
    movement_finished = true;
    return false;
  }

  State state;
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if(!transformation_systems_[indices_[i].first]->getCurrentState(indices_[i].second, state))
    {
      return false;
    }
    state.get(desired_positions(i), desired_velocities(i), desired_accelerations(i));
  }

  // only integrate the canonical system when movement hasn't finished yet...
  if(state_->num_generated_samples_+1 < num_samples)
  {
    state_->num_generated_samples_++;
    if (!canonical_system_->integrate(state_->current_time_))
    {
      Logger::logPrintf("Problem while integrating the canonical system. (Real-time violation).", Logger::ERROR);
      movement_finished = true;
      return false;
    }
  }
  else
  {
    canonical_system_->getState()->setCanX(0.0);
    state_->is_start_set_ = false;
    state_->is_setup_ = false;
    movement_finished = true;
  }

  // integrate progress indicator in any case
  canonical_system_->integrateProgress(state_->current_time_);

  return true;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::propagateStep(VectorXd& desired_positions,
                                             VectorXd& desired_velocities,
                                             VectorXd& desired_accelerations,
                                             bool& movement_finished,
                                             const double sampling_duration,
                                             const int num_samples)
{
  return propagateStep(desired_positions, desired_velocities, desired_accelerations, movement_finished, zero_feedback_, sampling_duration, num_samples);
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::propagateStep(VectorXd& desired_positions,
                                             VectorXd& desired_velocities,
                                             VectorXd& desired_accelerations,
                                             bool& movement_finished,
                                             const Eigen::VectorXd& feedback)
{
  assert(initialized_);
  int num_samples = 0;
  state_->current_time_.getNumberOfIntervalSteps(num_samples);
  double sampling_duration = state_->current_time_.getTau();
  return propagateStep(desired_positions, desired_velocities, desired_accelerations, movement_finished, feedback, sampling_duration, num_samples);
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::propagateStep(VectorXd& desired_positions,
                                             VectorXd& desired_velocities,
                                             VectorXd& desired_accelerations,
                                             bool& movement_finished)
{
  return propagateStep(desired_positions, desired_velocities, desired_accelerations, movement_finished, zero_feedback_);
}

//bool DynamicMovementPrimitive::getAllLWRParmameters(vector<lwr_lib::LWRParamConstPtr>& lwr_parameters) const
//{
//    assert(initialized_);
//    lwr_parameters.clear();
//    for (int i = 0; i < getNumberOfTransformationSystems(); ++i)
//    {
//        lwr_parameters.push_back(transformation_systems_[i]->parameters_->lwr_model_->getParameters());
//    }
//    return true;
//}

//bool DynamicMovementPrimitive::getLWRParmameters(lwr_lib::LWRParameters& lwr_parameters, int index) const
//{
//    assert(initialized_);
//    assert((index >= 0) && (index < getNumberOfTransformationSystems()));
//    lwr_parameters = *(transformation_systems_[index]->parameters_->lwr_model_->getParameters());
//    return true;
//}

bool DynamicMovementPrimitive::getNumRFS(vector<int>& num_rfs) const
{
  assert(initialized_);
  num_rfs.clear();
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    num_rfs.push_back(transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getNumRFS());
  }
  return true;
}

bool DynamicMovementPrimitive::getThetas(vector<VectorXd>& thetas) const
{
  assert(initialized_);
  thetas.clear();
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    int num_rfs = transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getNumRFS();
    VectorXd theta_vector = VectorXd::Zero(num_rfs);
    if (!transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getThetas(theta_vector))
    {
      Logger::logPrintf("Could not retrieve thetas from transformation system >%i<.", Logger::ERROR, indices_[i].first);
      return false;
    }
    thetas.push_back(theta_vector);
  }
  return true;
}

bool DynamicMovementPrimitive::setThetas(const vector<VectorXd>& thetas)
{
  assert(initialized_);
  if(static_cast<int> (thetas.size()) != getNumDimensions())
  {
    Logger::logPrintf("Size of theta vector >%i< does not match number of DMP dimensions. Cannot set thetas.",
                      Logger::ERROR, (int)thetas.size(), getNumDimensions());
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->setThetas(thetas[i]))
    {
      Logger::logPrintf("Could not set thetas of transformation system >%i<.", Logger::ERROR, indices_[i].first);
      return false;
    }
  }
  return true;
}

bool DynamicMovementPrimitive::generateBasisFunctionMatrix(const int num_time_steps, vector<MatrixXd>& basis_functions) const
{
  assert(initialized_);
  if(num_time_steps <= 0)
  {
    Logger::logPrintf("Number of time steps must >%i< be possitive. Cannot generate basis function matrix.", Logger::ERROR, num_time_steps);
    return false;
  }

  VectorXd input_vector;
  if(!canonical_system_->getRollout(num_time_steps, parameters_->cutoff_, input_vector))
  {
    return false;
  }

  basis_functions.clear();
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    int num_rfs = transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getNumRFS();
    MatrixXd basis_function_matrix = MatrixXd::Zero(num_time_steps, num_rfs);
    // generate basis function matrix evaluated using the canonical system vector
    if(!transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->generateBasisFunctionMatrix(input_vector, basis_function_matrix))
    {
      return false;
    }
    // multiply by the canonical system vector
    for (int j = 0; j < num_rfs; ++j)
    {
      basis_function_matrix.col(j) = (basis_function_matrix.col(j).array() * input_vector.array()).matrix();
    }
    basis_functions.push_back(basis_function_matrix);
  }

  return true;
}

bool DynamicMovementPrimitive::getBasisFunctionCenters(vector<VectorXd>& basis_function_centers) const
{
  assert(initialized_);
  basis_function_centers.clear();
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    int num_rfs = transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getNumRFS();
    VectorXd widths = VectorXd::Zero(num_rfs);
    VectorXd centers = VectorXd::Zero(num_rfs);
    if(!transformation_systems_[indices_[i].first]->parameters_[indices_[i].second]->lwr_model_->getWidthsAndCenters(widths, centers))
    {
      return false;
    }
    basis_function_centers.push_back(centers);
  }
  return true;
}

bool DynamicMovementPrimitive::getCanonicalSystemState(double& x,
                                                       double& xd) const
{
  assert(initialized_);
  x = canonical_system_->state_->getStateX();
  xd = canonical_system_->state_->getStateXd();
  return true;
}

}
