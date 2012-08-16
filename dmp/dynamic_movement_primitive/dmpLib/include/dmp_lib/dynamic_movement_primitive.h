/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		dynamic_movement_primitive.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 3, 2010

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_

// system include
#include <Eigen/Eigen>
#include <vector>

#include <boost/shared_ptr.hpp>

// local include
#include <dmp_lib/transformation_system.h>
#include <dmp_lib/canonical_system.h>

#include <dmp_lib/dynamic_movement_primitive_parameters.h>
#include <dmp_lib/dynamic_movement_primitive_state.h>

#include <dmp_lib/trajectory.h>
#include <dmp_lib/status.h>
#include <dmp_lib/logger.h>

namespace dmp_lib
{

/*!
 */
class DynamicMovementPrimitive : public Status
{

public:

  /*! Constructor
   */
  DynamicMovementPrimitive() {};

  /*! Destructor
   */
  virtual ~DynamicMovementPrimitive() {};

  /*!
   * @param params
   * @return True if equal, otherwise False
   */
  bool operator==(const DynamicMovementPrimitive &dmp) const
  {
    if ((isInitialized() && dmp.isInitialized())
        && (*parameters_ == *(dmp.parameters_))
        && (*state_ == *(dmp.state_))
        && (*canonical_system_ == *(dmp.canonical_system_))
        && (indices_ == dmp.indices_)
        && (debug_dimensions_ == dmp.debug_dimensions_)
        && (zero_feedback_ == dmp.zero_feedback_)
        && (transformation_systems_.size() == dmp.transformation_systems_.size()))
    {
      for (unsigned int i=0; i < transformation_systems_.size(); ++i)
      {
        if (*(transformation_systems_[i]) != *(dmp.transformation_systems_[i]))
        {
          return false;
        }
      }
      return true;
    }
    return false;
  }
  bool operator!=(const DynamicMovementPrimitive &dmp) const
  {
    return !(*this == dmp);
  }

  /*!
   * @param parameters
   * @param state
   * @param transformation_systems
   * @param canonical_system
   * @return True on success, otherwise False
   */
  bool initialize(DMPParamPtr parameters,
                  DMPStatePtr state,
                  std::vector<TSPtr>& transformation_systems,
                  CSPtr canonical_system);

  /*!
   * @param parameters
   * @param canonical_system
   * @param transformation_systems
   * @return True on success, otherwise False
   */
  bool initialize(DMPParamPtr parameters,
                  std::vector<TSPtr>& transformation_systems,
                  CSPtr canonical_system);

  /*!
   * @param other_dmp
   * @return
   */
  bool isCompatible(const DynamicMovementPrimitive& other_dmp) const;

  /*! Changes the type of the DMP
   * @param type
   */
  void changeType(const int type);

  /*!
   * @param type
   * @return
   */
  bool hasType(const int type) const;

  /*!
   * @return
   */
  int getType() const;

  /*!
   * @return the sequence number used for communication between the dmp controller
   * and the dmp client
   */
  int getSeq() const;

  /*! Sets the id of the DMP which will get stored/remembered
   * @return
   */
  void setId(const int id);

  /*! Gets the id of the DMP
   * @return
   */
  int getId() const;

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool get(DMPParamConstPtr& parameters,
           DMPStateConstPtr& state) const;

  /*!
   * @param trajectory
   * @param debug_trajectory
   * @return True on success, otherwise False
   */
  bool learnFromTrajectory(const Trajectory& trajectory,
                           TrajectoryPtr debug_trajectory = TrajectoryPtr());

  /*! Generates a minimum jerk function and learns the DMP. Note: so far, quaternions are not
   * handled.
   * @param start
   * @param goal
   * @param duration
   * @param delta_t
   * @param debug_trajectory
   * @return True on success, otherwise False
   */
  bool learnFromMinimumJerk(const Eigen::VectorXd& start,
                            const Eigen::VectorXd& goal,
                            const double sampling_frequency,
                            const double initial_duration,
                            TrajectoryPtr debug_trajectory = TrajectoryPtr());
  bool learnFromMinimumJerk(const std::vector<double>& start,
                            const std::vector<double>& goal,
                            const double sampling_frequency,
                            const double initial_duration,
                            TrajectoryPtr debug_trajectory = TrajectoryPtr());

  /*!
   * @param waypoints
   * @param sampling_frequency
   * @param initial_durations
   * @param debug_trajectory
   * @return
   */
  bool learnFromMinimumJerk(const std::vector<Eigen::VectorXd>& waypoints,
                            const double sampling_frequency,
                            const std::vector<double>& initial_durations,
                            TrajectoryPtr debug_trajectory = TrajectoryPtr());
  bool learnFromMinimumJerk(const std::vector<std::vector<double> >& waypoints,
                            const double sampling_frequency,
                            const std::vector<double>& initial_durations,
                            TrajectoryPtr debug_trajectory = TrajectoryPtr());


  /*!
   * @param thetas
   * @param initial_start
   * @param initial_goal
   * @param sampling_frequency
   * @param initial_duration
   * @return True on success, otherwise False
   */
  bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas,
                       const Eigen::VectorXd& initial_start,
                       const Eigen::VectorXd& initial_goal,
                       const double sampling_frequency,
                       const double initial_duration);
  bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas,
                       const std::vector<double>& initial_start,
                       const std::vector<double>& initial_goal,
                       const double sampling_frequency,
                       const double initial_duration);

  /*! Sets up the DMP.
   * @param start
   * @param goal
   * @param movement_duration
   * @param sampling_frequency
   * @return True on success, otherwise False
   */
  bool setup(const Eigen::VectorXd& start,
             const Eigen::VectorXd& goal,
             const double movement_duration,
             const double sampling_frequency);

  /*! Sets up the DMP. The sampling frequency is set to
   * the frequency at which the DMP has been learned.
   * @param start
   * @param goal
   * @param movement_duration
   * @return True on success, otherwise False
   */
  bool setupDuration(const Eigen::VectorXd& start,
                     const Eigen::VectorXd& goal,
                     const double movement_duration);

  /*! Sets up the DMP. The movement duration is set to the
   * initial movement duration.
   * @param start
   * @param goal
   * @param sampling_frequency
   * @return True on success, otherwise False
   */
  bool setupSamplingFrequency(const Eigen::VectorXd& start,
                              const Eigen::VectorXd& goal,
                              const double sampling_frequency);

  /*! Sets up the DMP. Start is set to the initial start.
   * @param goal
   * @param movement_duration
   * @param sampling_frequency
   * @return True on success, otherwise False
   */
  bool setup(const Eigen::VectorXd& goal,
             const double movement_duration,
             const double sampling_frequency);

  /*! Sets up the DMP. Start is set to the initial start, and
   * sampling frequency is set to the frequency at which the DMP has been learned.
   * @param goal
   * @param movement_duration
   * @return True on success, otherwise False
   */
  bool setupDuration(const Eigen::VectorXd& goal,
                     const double movement_duration);

  /*! Sets up the DMP. Start is set to the initial start, and
   * movement duration is set to the initial movement duration.
   * @param goal
   * @param sampling_frequency
   * @return True on success, otherwise False
   */
  bool setupSamplingFrequency(const Eigen::VectorXd& goal,
                              const double sampling_frequency);

  /*! Sets up the DMP. Start is set to the initial start, and
   * movement duration is set to the sampling frequency at which the DMP has been learned.
   * @param sampling_frequency
   * @return True on success, otherwise False
   */
  bool setupSamplingFrequency(const double sampling_frequency);

  /*! Sets up the DMP. Start and goal are set to the initial start and goal,
   * sampling frequency is set to the frequency at which the DMP has been learned.
   * @param movement_duration
   * @return True on success, otherwise False
   */
  bool setupDuration(const double movement_duration);

  /*! Sets up the DMP. Start is set to the initial start, and
   * movement duration and sampling frequency are set to the initial movement duration
   * and the sampling frequency at which the DMP has been learned.
   * @param goal
   * @return True on success, otherwise False
   */
  bool setup(const Eigen::VectorXd& goal);

  /*! Sets up the DMP. Start and goal are set to the initial start and goal, and
   * movement duration and sampling frequency are set to the initial movement duration
   * and the sampling frequency at which the DMP has been learned.
   * @return True on success, otherwise False
   */
  bool setup();

  /*!
   * @return True if DMP is setup, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool isSetup() const;

  /*! Sets the "is_setup" and "is_start_set" flag to false
   * REAL-TIME REQUIREMENTS
   */
  void unset();

  /*!
   * @return True if start of the DMP is set, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool isStartSet() const;

  /*! Gets the current position of the dmp
   * @param current_desired_position
   * @param start_index
   * @param end_index
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getCurrentPosition(Eigen::VectorXd& current_desired_position,
                          const int start_index,
                          const int end_index);

  /*! Sets the goal of the the transformation system indesetxed by start_index to
   * end_index to the new_goal
   * @param new_goal
   * @param start_index
   * @param end_index
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool changeGoal(const Eigen::VectorXd& new_goal,
                  const int start_index,
                  const int end_index);

  /*!
   * @param new_goal
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool changeGoal(const Eigen::VectorXd& new_goal);

  /*! Sets the goal of the transformation system indexed by index to new_goal
   * @param new_goal
   * @param index
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool changeGoal(const double new_goal,
                  const int index);

  /*! Sets the start of all transformation systems to new_start
   * @param new_start
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool changeStart(const Eigen::VectorXd& new_start);

  /*! Propagates the DMP and generates an entire rollout of size num_samples. The duration of the DMP need to be
   *  set previously using one of the setup functions. The sampling duration and the number of samples specified
   *  determine the length of the trajectory and its sampling frequecy. Note, the sampling frequency of the
   *  trajectory may change.
   * @param trajectory
   * @param sampling_duration
   * @param num_samples
   * @return True on success, otherwise False
   */
  bool propagateFull(Trajectory& trajectory,
                     const double sampling_duration,
                     const int num_samples);

  /*! Propagates the DMP and generates an entire rollout. The DMP needs to be setup using one
   *  of the setup functions. The sampling duration and the number of samples specified
   *  determine the length of the trajectory and its sampling frequecy. The number of samples is
   *  specified by the sampling duration and the sampling frequency at which the DMP
   *  has been learned. Note, the sampling frequency of the trajectory may change.
   * @param trajectory
   * @param sampling_duration
   * @return
   */
  bool propagateFull(Trajectory& trajectory,
                     const double sampling_duration);

  /*!
   * @param desired_positions
   * @param desired_velocities
   * @param desired_accelerations
   * @param movement_finished
   * @param feedback
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool propagateStep(Eigen::VectorXd& desired_positions,
                     Eigen::VectorXd& desired_velocities,
                     Eigen::VectorXd& desired_accelerations,
                     bool& movement_finished,
                     const Eigen::VectorXd& feedback);
  bool propagateStep(Eigen::VectorXd& desired_positions,
                     Eigen::VectorXd& desired_velocities,
                     Eigen::VectorXd& desired_accelerations,
                     bool& movement_finished);

  /*!
   * @param desired_positions
   * @param desired_velocities
   * @param desired_accelerations
   * @param movement_finished
   * @param sampling_duration
   * @param num_samples
   * @param feedback
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool propagateStep(Eigen::VectorXd& desired_positions,
                     Eigen::VectorXd& desired_velocities,
                     Eigen::VectorXd& desired_accelerations,
                     bool& movement_finished,
                     const Eigen::VectorXd& feedback,
                     const double sampling_duration,
                     const int num_samples);
  bool propagateStep(Eigen::VectorXd& desired_positions,
                     Eigen::VectorXd& desired_velocities,
                     Eigen::VectorXd& desired_accelerations,
                     bool& movement_finished,
                     const double sampling_duration,
                     const int num_samples);

  /*!
   * @return
   */
  double getProgress() const;

  /*!
   * @param initial_duration
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getInitialDuration(double& initial_duration) const;

  /*!
   * @param initial_sampling_frequency
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getInitialSamplingFrequency(double& initial_sampling_frequency) const;

  /*!
   * @param sampling_frequency
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getSamplingFrequency(double& sampling_frequency) const;

  /*!
   * @param duration
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getDuration(double& duration) const;

  /*! Sets the initial start of the DMP (used during relative task frame computation)
   * @param initial_start
   * REAL-TIME REQUIREMENTS
   * @return True on success, otherwise False
   */
  bool setInitialStart(const Eigen::VectorXd& initial_start);

  /*! Sets the initial start of the DMP (used during relative task frame computation)
   * @param initial_start
   * REAL-TIME REQUIREMENTS
   * @return True on success, otherwise False
   */
  bool setInitialStart(const std::vector<double>& initial_start);

  /*!
   * @param initial_start
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getInitialStart(Eigen::VectorXd& initial_start) const;

  /*!
   * @param initial_start
   * @param in_real_time
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getInitialStart(std::vector<double>& initial_start,
                       bool in_real_time = true) const;

  /*! Sets the initial start of the DMP (used during relative task frame computation)
   * @param initial_goal
   * REAL-TIME REQUIREMENTS
   * @return True on success, otherwise False
   */
  bool setInitialGoal(const Eigen::VectorXd& initial_goal);

  /*! Sets the initial start of the DMP (used during relative task frame computation)
   * @param initial_goal
   * @param in_real_time
   * REAL-TIME REQUIREMENTS
   * @return True on success, otherwise False
   */
  bool setInitialGoal(const std::vector<double>& initial_goal);

  /*!
   * @param initial_start
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getInitialGoal(Eigen::VectorXd& initial_goal) const;

  /*!
   * @param initial_goal
   * @param in_real_time
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getInitialGoal(std::vector<double>& initial_goal,
                      bool in_real_time = true) const;

  /*!
   * @param start
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getStart(Eigen::VectorXd& goal) const;

  /*!
   * @param start
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getStart(std::vector<double>& start,
                bool in_real_time = true) const;

  /*!
   * @param variable_names
   * @param start
   * @return True on success, otherwise False
   */
  bool getStart(const std::vector<std::string> variable_names,
                std::vector<double> &start) const;

  /*!
   * @param goal
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getGoal(Eigen::VectorXd& goal) const;

  /*!
   * @param goal
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getGoal(std::vector<double>& goal,
               bool in_real_time = true) const;

  /*!
   * @param variable_names
   * @param goal
   * @return True on success, otherwise False
   */
  bool getGoal(const std::vector<std::string> variable_names,
               std::vector<double> &goal) const;

  //    /*!
  //     * @param lwr_parameters
  //     * @return True on success, otherwise False
  //     */
  //    bool getAllLWRParmameters(std::vector<lwr_lib::LWRParamConstPtr>& lwr_parameters) const;

  //    /*!
  //     * @param lwr_parameters
  //     * @param index
  //     * @return True on success, otherwise False
  //     */
  //    bool getLWRParmameters(lwr_lib::LWRParameters& lwr_parameters, int index) const;

  /*! Gets number of receptive fields of each transformation system dimension
   * @param num_rfs
   * @return True on success, otherwise False
   */
  bool getNumRFS(std::vector<int>& num_rfs) const;

  /*!
   * Gets the parameters of each transformation system dimension
   * @param thetas
   * @return True on success, otherwise False
   */
  bool getThetas(std::vector<Eigen::VectorXd>& thetas) const;

  /*!
   * Sets the parameters of each transformation system dimension
   * @param thetas
   * @return True on success, otherwise False
   */
  bool setThetas(const std::vector<Eigen::VectorXd>& thetas);

  /*! Sets the basis functions provided the generated input vector using the current
   * parameters of the canonical system of length num_time_steps.
   * @param num_time_steps
   * @param basis_functions
   * @return True on success, otherwise False
   */
  bool generateBasisFunctionMatrix(const int num_time_steps, std::vector<Eigen::MatrixXd>& basis_functions) const;

  /*! Gets the centers of the basis functions
   * @param basis_function_centers
   * @return True on success, otherwise False
   */
  bool getBasisFunctionCenters(std::vector<Eigen::VectorXd>& basis_function_centers) const;

  /*!
   * @return Variable names of each transformation system dimension contained in the DMP
   */
  std::vector<std::string> getVariableNames() const;

  /*!
   * @param x
   * @param xd
   * @return True on success, otherwise False
   */
  bool getCanonicalSystemState(double& x,
                               double& xd) const;

  /*! Returns the number of transformation systems. Note: a transformation system can
   * contain multiple dimensions.
   * @return
   */
  int getNumTransformationSystems() const;

  /*! Returns the number of transformation systems dimension. If each transformation system
   * only contains one dimension, this is equal to getNumTransformationSystems()
   * @return
   */
  int getNumDimensions() const;

  /*!
   */
  virtual std::string getVersionString() const = 0;

  /*!
   * @return Pointer to internal parameters (handle with care).
   * TODO: Think about changing this
   */
  DMPParamPtr getParameters();

  /*!
   * @return Pointer to internal parameters (handle with care).
   * TODO: Think about changing this
   * REAL-TIME REQUIREMENTS
   */
  DMPStatePtr getState();

  /*!
   * @return Pointer to internal parameters (handle with care).
   * TODO: Think about changing this
   */
  std::vector<TSPtr> getTransformationSystem() const;

  /*!
   * @param
   * @return
   * REAL-TIME REQUIREMENTS
   */
  TSPtr getTransformationSystem(const int index) const;

  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  CSPtr getCanonicalSystem() const;

  /*!
   * @return
   */
  const std::vector<std::pair<int, int> >& getIndices() const
  {
    return indices_;
  }

protected:

  /*!
   */
  DMPParamPtr parameters_;

  /*!
   */
  DMPStatePtr state_;

  /*!
   */
  std::vector<TSPtr> transformation_systems_;

  /*!
   */
  CSPtr canonical_system_;

  /*!
   * @return
   */
  bool setupIndices();

  /*! Contains the mapping from dimension index to transformation system index and dimension index
   */
  std::vector<std::pair<int, int> > indices_;

private:

  /*!
   * @param trajectory
   * @return True on success, otherwise False
   */
  bool prepareTrajectory(Trajectory& trajectory);

  /*!
   * @return True on success, otherwise False
   */
  bool learnTransformationTarget();

  /*!
   * @return
   */
  bool isReadyToPropagate();

  /*!
   * @param dt_total
   * @param num_iteration
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool integrate(const int num_iteration, const Eigen::VectorXd& feedback);

  /*!
   * @param debug_trajectory
   * @return
   */
  bool createDebugTrajectory(Trajectory& debug_trajectory, const Trajectory& trajectory);
  bool logDebugTrajectory(Trajectory& debug_trajectory);
  std::vector<int> debug_dimensions_;

  Eigen::VectorXd zero_feedback_;

};

/*! Abbreviation for convinience
 */
typedef boost::shared_ptr<DynamicMovementPrimitive> DMPPtr;
typedef boost::shared_ptr<DynamicMovementPrimitive const> DMPConstPtr;

// Inline function definitions
inline int DynamicMovementPrimitive::getNumTransformationSystems() const
{
  assert(initialized_);
  return static_cast<int> (transformation_systems_.size());
}

inline int DynamicMovementPrimitive::getNumDimensions() const
{
  assert(initialized_);
  return static_cast<int> (indices_.size());
}

inline DMPParamPtr DynamicMovementPrimitive::getParameters()
{
  assert(initialized_);
  return parameters_;
}

inline DMPStatePtr DynamicMovementPrimitive::getState()
{
  assert(initialized_);
  return state_;
}

// REAL-TIME REQUIREMENTS
inline double DynamicMovementPrimitive::getProgress() const
{
  assert(initialized_);
  if(state_->current_time_.getTau() > 0)
    return canonical_system_->getProgressTime() / state_->current_time_.getTau();
  return -1.0;
}

inline std::vector<TSPtr> DynamicMovementPrimitive::getTransformationSystem() const
{
  assert(initialized_);
  return transformation_systems_;
}

inline TSPtr DynamicMovementPrimitive::getTransformationSystem(const int index) const
{
  assert(initialized_);
  assert(index >= 0);
  assert(index < getNumTransformationSystems());
  return transformation_systems_[index];
}

inline CSPtr DynamicMovementPrimitive::getCanonicalSystem() const
{
  assert(initialized_);
  return canonical_system_;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::isSetup() const
{
  assert(initialized_);
  return state_->is_setup_;
}

// REAL-TIME REQUIREMENTS
inline void DynamicMovementPrimitive::unset()
{
  assert(initialized_);
  state_->is_setup_ = false;
  state_->is_start_set_ = false;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::isStartSet() const
{
  assert(initialized_);
  return state_->is_start_set_;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getCurrentPosition(Eigen::VectorXd& current_desired_position,
                                                         const int start_index,
                                                         const int end_index)
{
  assert(initialized_);
  if ((!state_->is_setup_) || (start_index < 0) || (end_index > getNumDimensions()) || (end_index <= start_index))
  {
    return false;
  }
  if (current_desired_position.size() != end_index - start_index)
  {
    Logger::logPrintf("Provided vector has wrong size >%i<, required size is >%i<. Cannot get current position (Real-time violation).", Logger::ERROR,
                      current_desired_position.size(), end_index - start_index);
    return false;
  }
  State current_state;
  for (int i = start_index; i < end_index; ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getCurrentState(indices_[i].second, current_state))
    {
      return false;
    }
    current_desired_position(i) = current_state.getX();
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::changeGoal(const Eigen::VectorXd& new_goal,
                                                 const int start_index,
                                                 const int end_index)
{
  assert(initialized_);
  // if ((!state_->is_setup_) || (start_index < 0) || (end_index > getNumDimensions()) || (end_index <= start_index))
  if ((start_index < 0) || (end_index > getNumDimensions()) || (end_index <= start_index))
  {
    return false;
  }
  if (new_goal.size() != end_index - start_index)
  {
    Logger::logPrintf("Provided vector has wrong size >%i<, required size is >%i<. Cannot change goal position (Real-time violation).", Logger::ERROR,
                      new_goal.size(), end_index - start_index);
    return false;
  }
  for (int i = start_index; i < end_index; ++i)
  {
    if (!transformation_systems_[indices_[i].first]->setGoal(indices_[i].second, new_goal(i - start_index)))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::changeGoal(const Eigen::VectorXd& new_goal)
{
  assert(initialized_);
  return changeGoal(new_goal, 0, getNumDimensions());
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::changeGoal(const double new_goal,
                                                 const int index)
{
  assert(initialized_);
  // if ((!state_->is_setup_) || (index < 0) || (index >= getNumDimensions()))
  if ((index < 0) || (index >= getNumDimensions()))
  {
//    if(!state_->is_setup_)
//    {
//      Logger::logPrintf("DMP is not setup (Real-time violation).", Logger::ERROR);
//    }
    if(index < 0)
    {
      Logger::logPrintf("index < 0 (Real-time violation).", Logger::ERROR);
    }
    if(index >= getNumDimensions())
    {
      Logger::logPrintf("index >= getNumDimensions() (Real-time violation).", Logger::ERROR);
    }

    return false;
  }
  if (!transformation_systems_[indices_[index].first]->setGoal(indices_[index].second, new_goal))
  {
    Logger::logPrintf("!transformation_systems_[indices_[index].first]->setGoal(indices_[index].second, new_goal) (Real-time violation).", Logger::ERROR);

    return false;
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::changeStart(const Eigen::VectorXd& new_start)
{
  assert(initialized_);
  if (!state_->is_setup_)
  {
    Logger::logPrintf("DMP is not setup (Real-time violation).", Logger::ERROR);
    return false;
  }
  if ((int)new_start.size() < getNumDimensions())
  {
    Logger::logPrintf("Start vector has wrong size >%i<, it should be >%i< (Real-time violation).", Logger::ERROR, new_start.size(), getNumDimensions());
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->setStart(indices_[i].second, new_start(i)))
    {
      return false;
    }
    // set current state to start state (zero velocity and acceleration)
    if (!transformation_systems_[indices_[i].first]->setCurrentState(indices_[i].second, State(new_start(i), 0.0, 0.0)))
    {
      return false;
    }
  }
  return (state_->is_start_set_ = true);
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::setInitialStart(const Eigen::VectorXd& initial_start)
{
  assert(initialized_);
  if ((int)initial_start.size() < getNumDimensions())
  {
    Logger::logPrintf("Invalid vector size >%i<. It should be >%i<. Cannot set initial start. (Real-time violation).",
                      Logger::ERROR, (int)initial_start.size(), getNumDimensions());
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->setInitialStart(indices_[i].second, initial_start(i)))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::setInitialStart(const std::vector<double>& initial_start)
{
  assert(initialized_);
  if ((int)initial_start.size() < getNumDimensions())
  {
    Logger::logPrintf("Invalid vector size >%i<. It should be >%i<. Cannot set initial start. (Real-time violation).",
                      Logger::ERROR, (int)initial_start.size(), getNumDimensions());
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->setInitialStart(indices_[i].second, initial_start[i]))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getInitialStart(Eigen::VectorXd& initial_start) const
{
  assert(initialized_);
  if ((int)initial_start.size() < getNumDimensions())
  {
    Logger::logPrintf("Invalid vector size >%i<. Cannot get initial start. (Real-time violation).", Logger::ERROR, (int)initial_start.size());
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getInitialStart(indices_[i].second, initial_start(i)))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getInitialStart(std::vector<double>& initial_start,
                                                      bool in_real_time) const
{
  assert(initialized_);
  if ((int)initial_start.size() < getNumDimensions())
  {
    if (in_real_time)
    {
      Logger::logPrintf("Invalid vector size >%i<. Cannot get initial start. (Real-time violation).", Logger::ERROR, (int)initial_start.size());
      return false;
    }
    Logger::logPrintf("Resizing array to store start of the DMP.", Logger::DEBUG);
    initial_start.resize(getNumDimensions());
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getInitialStart(indices_[i].second, initial_start[i]))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::setInitialGoal(const Eigen::VectorXd& initial_goal)
{
  assert(initialized_);
  if ((int)initial_goal.size() < getNumDimensions())
  {
    Logger::logPrintf("Invalid vector size >%i<. It should be >%i<. Cannot set initial goal. (Real-time violation).",
                      Logger::ERROR, (int)initial_goal.size(), getNumDimensions());
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->setInitialGoal(indices_[i].second, initial_goal(i)))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::setInitialGoal(const std::vector<double>& initial_goal)
{
  assert(initialized_);
  if ((int)initial_goal.size() < getNumDimensions())
  {
    Logger::logPrintf("Invalid vector size >%i<. It should be >%i<. Cannot set initial goal. (Real-time violation).",
                      Logger::ERROR, (int)initial_goal.size(), getNumDimensions());
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->setInitialGoal(indices_[i].second, initial_goal[i]))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getInitialGoal(Eigen::VectorXd &initial_goal) const
{
  assert(initialized_);
  if (initial_goal.size() < getNumDimensions())
  {
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getInitialGoal(indices_[i].second, initial_goal(i)))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getInitialGoal(std::vector<double> &initial_goal,
                                                     bool in_real_time) const
{
  assert(initialized_);
  if ((int)initial_goal.size() < getNumDimensions())
  {
    if (in_real_time)
    {
      return false;
    }
    Logger::logPrintf("Resizing array to store goal of the DMP.", Logger::DEBUG);
    initial_goal.resize(getNumDimensions());
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getInitialGoal(indices_[i].second, initial_goal[i]))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getStart(Eigen::VectorXd &start) const
{
  assert(initialized_);
  if (start.size() < getNumDimensions())
  {
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getStart(indices_[i].second, start(i)))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getStart(std::vector<double> &start,
                                               bool in_real_time) const
{
  assert(initialized_);
  if ((int)start.size() < getNumDimensions())
  {
    if (in_real_time)
    {
      return false;
    }
    Logger::logPrintf("Resizing array to store goal of the DMP.", Logger::DEBUG);
    start.resize(getNumDimensions());
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getStart(indices_[i].second, start[i]))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getGoal(Eigen::VectorXd &goal) const
{
  assert(initialized_);
  if (goal.size() < getNumDimensions())
  {
    return false;
  }
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getGoal(indices_[i].second, goal(i)))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getGoal(std::vector<double> &goal,
                                              bool in_real_time) const
{
  assert(initialized_);
  if ((int)goal.size() < getNumDimensions())
  {
    if (in_real_time)
    {
      return false;
    }
    Logger::logPrintf("Resizing array to store goal of the DMP.", Logger::DEBUG);
    goal.resize(getNumDimensions());
  }
  for (int i = 0; i < (int)goal.size(); ++i)
  {
    if (!transformation_systems_[indices_[i].first]->getGoal(indices_[i].second, goal[i]))
    {
      return false;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getInitialDuration(double& initial_duration) const
{
  assert(initialized_);
  initial_duration = parameters_->initial_time_.getTau();
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getInitialSamplingFrequency(double& initial_sampling_frequency) const
{
  assert(initialized_);
  if (!parameters_->initial_time_.getSamplingFrequency(initial_sampling_frequency))
  {
    return false;
  }
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getDuration(double& duration) const
{
  assert(initialized_);
  duration = state_->current_time_.getTau();
  return true;
}

// REAL-TIME REQUIREMENTS
inline bool DynamicMovementPrimitive::getSamplingFrequency(double& sampling_frequency) const
{
  assert(initialized_);
  if (!state_->current_time_.getSamplingFrequency(sampling_frequency))
  {
    return false;
  }
  return true;
}

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_ */
