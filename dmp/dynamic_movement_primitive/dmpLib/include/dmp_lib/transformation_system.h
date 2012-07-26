/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		transformation_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 4, 2010

 *********************************************************************/

#ifndef TRANSFORMATION_SYSTEM_BASE_H_
#define TRANSFORMATION_SYSTEM_BASE_H_

// system include
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

// local include
#include <dmp_lib/transformation_system_parameters.h>
#include <dmp_lib/transformation_system_state.h>
#include <dmp_lib/canonical_system_parameters.h>
#include <dmp_lib/canonical_system_state.h>
#include <dmp_lib/time.h>
#include <dmp_lib/state.h>
#include <dmp_lib/status.h>

namespace dmp_lib
{

/*!
 */
class TransformationSystem : public Status
{

  /*! Allow the DynamicMovementPrimitive class to access private member variables directly
   */
  friend class DynamicMovementPrimitive;

public:

  /*! Constructor
   */
  TransformationSystem() :
    integration_method_(NORMAL) {};

  /*! Destructor
   */
  virtual ~TransformationSystem() {};

  /*!
   * @param transformation_system
   * @return True if equal, otherwise False
   */
  bool operator==(const TransformationSystem &transformation_system) const
  {
    if ((isInitialized() && transformation_system.isInitialized())
        && (integration_method_ == transformation_system.integration_method_)
        && (parameters_.size() == transformation_system.parameters_.size())
        && (states_.size() == transformation_system.states_.size()))
    {
      for (unsigned int i = 0; i < parameters_.size(); ++i)
      {
        if (*(parameters_[i]) != *(transformation_system.parameters_[i]))
        {
          return false;
        }
      }
      for (unsigned int i = 0; i < states_.size(); ++i)
      {
        if (*(states_[i]) != *(transformation_system.states_[i]))
        {
          return false;
        }
      }
      return true;
    }
    return false;
  }
  bool operator!=(const TransformationSystem &transformation_system) const
  {
    return !(*this == transformation_system);
  }

  /*!
   */
  enum IntegrationMethod
  {
    NORMAL,         //!< NORMAL
    QUATERNION      //!< QUATERNION
  };

  /*! Initializes the transformation system. The parameters have to be initialized.
   * @param parameters
   * @param states
   * @return True if success, otherwise False
   */
  bool initialize(const std::vector<TSParamPtr> parameters,
                  const std::vector<TSStatePtr> states,
                  const IntegrationMethod integration_method);

  /*!
   * @param parameters
   * @param states
   * @return True if success, otherwise False
   */
  bool get(std::vector<TSParamConstPtr>& parameters,
           std::vector<TSStateConstPtr>& states,
           IntegrationMethod& integration_method) const;

  /*! Reset the transformation system
   */
  virtual void reset() = 0;

  /*! This function takes the target states, the state of the canonical system, and the duration tau
   * and fits the transformation system
   *
   * @param target_states
   * @param canonical_system_state
   * @return True if success, otherwise False
   */
  virtual bool integrateAndFit(const std::vector<State>& target_states,
                               const CSStatePtr canonical_system_state,
                               const Time& dmp_time) = 0;

  /*!
   * @param canonical_system_state
   * @param dmp_time
   * @param feedback
   * @param num_iterations
   * @return False if the lwr model could not come up with a prediction for various reasons, otherwise True.
   * REAL-TIME REQUIREMENTS
   */
  virtual bool integrate(const CSStatePtr canonical_system_state,
                         const Time& dmp_time,
                         const Eigen::VectorXd& feedback,
                         const int num_iterations = 1) = 0;

  /*!
   * @param index
   * @param current_state
   * @return True if success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool setCurrentState(const int index, const State& current_state);

  /*!
   * @param current_states
   * @return True if success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool setCurrentStates(const std::vector<State>& current_states);

  /*!
   * @param index
   * @param state
   * @return True if success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getCurrentState(const int index, State& state) const;

  /*!
   * @param states
   */
  void getCurrentStates(std::vector<State>& states) const;

  /*!
   * @param index
   * @param start
   * @param initial
   * @return True if success, otherwise False
   */
  bool setStart(const int index, const double start, bool initial = false);

  /*!
   * @param start
   * @param initial
   * @return True if success, otherwise False
   */
  bool setStart(const std::vector<double> start, bool initial = false);

  /*!
   * @param index
   * @param start
   * @param initial
   * @return True if success, otherwise False
   */
  bool getStart(const int index, double& start, bool initial = false) const;

  /*!
   * @param start
   * @param initial
   */
  void getStart(std::vector<double>& start, bool initial = false) const;

  /*!
   * @param index
   * @param goal
   * @param initial
   * @return True if success, otherwise False
   */
  bool setGoal(const int index, const double goal, bool initial = false);

  /*!
   * @param goal
   * @param initial
   * @return True if success, otherwise False
   */
  bool setGoal(const std::vector<double> goal, bool initial = false);

  /*!
   * @param index
   * @param goal
   * @param initial
   * @return True if success, otherwise False
   */
  bool getGoal(const int index, double& goal, bool initial = false) const;

  /*!
   * @param goal
   * @param initial
   * @return True if success, otherwise False
   */
  void getGoal(std::vector<double>& goal, bool initial = false) const;

  /*!
   * @param index
   * @param initial_start
   * @return True if success, otherwise False
   */
  bool setInitialStart(const int index, const double initial_start);

  /*!
   * @param initial_start
   * @return True if success, otherwise False
   */
  bool setInitialStart(const std::vector<double> initial_start);

  /*!
   * @param index
   * @param initial_start
   * @return True if success, otherwise False
   */
  bool getInitialStart(const int index, double& initial_start) const;

  /*!
   * @param initial_start
   */
  void getInitialStart(std::vector<double>& initial_start) const;

  /*!
   * @param index
   * @param initial_goal
   * @return True if success, otherwise False
   */
  bool setInitialGoal(const int index, const double initial_goal);

  /*!
   * @param initial_goal
   * @return True if success, otherwise False
   */
  bool setInitialGoal(const std::vector<double> initial_goal);

  /*!
   * @param index
   * @param initial_goal
   * @return True if success, otherwise False
   */
  bool getInitialGoal(const int index, double& initial_goal) const;

  /*!
   * @param initial_goal
   */
  void getInitialGoal(std::vector<double>& initial_goal) const;

  /*!
   * @return
   */
  int getNumDimensions() const;

  /*!
   * @param index
   * @param name
   * @return
   */
  bool getName(const int index, std::string& name) const;

  /*!
   * @param index
   * @return The name of the controlled variable, if index is invalid then "null" is returned
   * REAL-TIME REQUIREMENTS
   */
  const std::string& getName(const int index) const;

  /*!
   * @param names
   * @return
   */
  void getNames(std::vector<std::string>& names) const;

  /*!
   * @return
   */
  std::vector<TSParamPtr> getParameters() const;

  /*!
   * @return
   */
  std::vector<TSStatePtr> getStates() const;

  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  TSParamPtr getParameters(const int index) const;

  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  TSStatePtr getStates(const int index) const;

  /*! Sets the integration method. Default is NORMAL. Set to QUATERNION if transformation system encodes a quaternion.
   * @param integration_method
   * @return True on success, False otherwise
   */
  bool setIntegrationMethod(IntegrationMethod integration_method);

  /*! Gets the integration method used by the transformation system
   * @return
   */
  const IntegrationMethod& getIntegrationMethod() const
  {
    return integration_method_;
  }

protected:

  /*!
   */
  std::vector<TSParamPtr> parameters_;

  /*!
   */
  std::vector<TSStatePtr> states_;

  /*!
   */
  IntegrationMethod integration_method_;

};

/*! Abbreviation for convinience
 */
typedef boost::shared_ptr<TransformationSystem> TSPtr;
typedef boost::shared_ptr<TransformationSystem const> TSConstPtr;

// Inline definitions follow
// REAL-TIME REQUIREMENT
// TODO: Think about this again... number of dimension changes if transformation system encodes a quaternion, does it ?
inline int TransformationSystem::getNumDimensions() const
{
  assert(initialized_);
  assert(parameters_.size() == states_.size());
  return static_cast<int>(parameters_.size());
}

// REAL-TIME REQUIREMENT
inline std::vector<TSParamPtr> TransformationSystem::getParameters() const
{
  assert(initialized_);
  return parameters_;
}
// REAL-TIME REQUIREMENT
inline std::vector<TSStatePtr> TransformationSystem::getStates() const
{
  assert(initialized_);
  return states_;
}
// REAL-TIME REQUIREMENT
inline TSParamPtr TransformationSystem::getParameters(const int index) const
{
  assert(index >= 0);
  assert(index < getNumDimensions());
  return parameters_[index];
}
// REAL-TIME REQUIREMENT
inline TSStatePtr TransformationSystem::getStates(const int index) const
{
  assert(index >= 0);
  assert(index < getNumDimensions());
  return states_[index];
}

}

#endif /* TRANSFORMATION_SYSTEM_BASE_H_ */
