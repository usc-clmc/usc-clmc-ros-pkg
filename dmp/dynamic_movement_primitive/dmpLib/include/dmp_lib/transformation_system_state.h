/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		transformation_system_state.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 7, 2010

 *********************************************************************/

#ifndef TRANSFORMATION_SYSTEM_STATE_BASE_H_
#define TRANSFORMATION_SYSTEM_STATE_BASE_H_

// system includes
#include <vector>
#include <boost/shared_ptr.hpp>

// local includes
#include <dmp_lib/state.h>

namespace dmp_lib
{

/*!
 */
class TransformationSystemState
{

  /*! Allow the TransformationSystem class to access private member variables directly
   */
  friend class TransformationSystem;
  friend class DynamicMovementPrimitive;

public:

  /*! Constructor
   */
  TransformationSystemState() :
    start_(0), goal_(0), f_(0), ft_(0) {};

  /*! Destructor
   */
  virtual ~TransformationSystemState() {};

  /*!
    * @param params
    * @return True if equal, otherwise False
    */
  bool operator==(const TransformationSystemState &state) const
  {
    return ( (internal_ == state.internal_)
        && (target_ == state.target_)
        && (current_ == state.current_)
        && (fabs(start_ - state.start_) < EQUALITY_PRECISSION)
        && (fabs(goal_ - state.goal_) < EQUALITY_PRECISSION)
        && (fabs(f_ - state.f_) < EQUALITY_PRECISSION)
        && (fabs(ft_ - state.ft_) < EQUALITY_PRECISSION)
        && (function_input_ == state.function_input_)
        && (function_target_ == state.function_target_) );
  }
  bool operator!=(const TransformationSystemState &state) const
  {
    return !(*this == state);
  }

  /*!
   * @param internal
   * @param target
   * @param current
   * @param start
   * @param goal
   * @param f
   * @param ft
   * @param function_input
   * @param function_target
   * @return
   */
  bool set(const State& internal,
           const State& target,
           const State& current,
           const double start,
           const double goal,
           const double f,
           const double ft/*,
           const std::vector<double>& function_input,
           const std::vector<double>& function_target*/);

  /*!
   * @param internal
   * @param target
   * @param current
   * @param start
   * @param goal
   * @param f
   * @param ft
   * @param function_input
   * @param function_target
   * @return
   */
  bool get(State& internal,
           State& target,
           State& current,
           double& start,
           double& goal,
           double& f,
           double& ft/*,
           std::vector<double>& function_input,
           std::vector<double>& function_target*/) const;

  /*!
   */
  void reset();

  /*!
   * @param internal_state
   */
  void setInternalState(const State& internal_state);

  /*!
   * @return
   */
  State getInternalState() const;

  /*!
   * @return
   */
  double getInternalStateX() const;

  /*!
   * @return
   */
  double getInternalStateXd() const;
  double getInternalStateXdd() const;

  /*!
   * @param target_state
   */
  void setTargetState(const State& target_state);

  /*!
   * @return
   */
  State getTargetState() const;

  /*!
   * @return
   */
  double getTargetStateX() const;

  /*!
   * @return
   */
  double getTargetStateXd() const;
  double getTargetStateXdd() const;

  /*!
   * @param current_state
   */
  void setCurrentState(const State& current_state);

  /*!
   * @return
   */
  State getCurrentState() const;

  /*!
   * @return
   */
  double getCurrentStateX() const;

  /*!
   * @return
   */
  double getCurrentStateXd() const;
  double getCurrentStateXdd() const;

  /*!
   */
  void setStart(const double start);

  /*!
   */
  double getStart() const;

  /*!
   */
  void setGoal(const double goal);

  /*!
   */
  double getGoal() const;

  /*!
   * @param ft
   */
  void setFT(const double ft);

  /*!
   * @return
   */
  double getFT() const;

  /*!
   * @param f
   */
  void setF(const double f);

  /*!
   * @return
   */
  double getF() const;

  /*!
   * @param f
   */
  //    void addFunctionInput(const double f);
  //
  //    std::vector<double> getFunctionInput() const;


protected:

  static const double EQUALITY_PRECISSION = 1e-6;

  /*!
   */
  State internal_;

  /*!
   */
  State target_;

  /*!
   */
  State current_;

  /*!
   */
  double start_;
  double goal_;

  /*!
   */
  double f_;
  double ft_;

  /*!
   */
  std::vector<double> function_input_;
  std::vector<double> function_target_;

private:

};

/*! Abbreviation for convinience
 */
typedef TransformationSystemState TSState;
typedef boost::shared_ptr<TSState> TSStatePtr;
typedef boost::shared_ptr<TSState const> TSStateConstPtr;

// inline functions
inline void TransformationSystemState::setInternalState(const State& internal_state)
{
  internal_ = internal_state;
}
inline State TransformationSystemState::getInternalState() const
{
  return internal_;
}
inline double TransformationSystemState::getInternalStateX() const
{
  return internal_.getX();
}
inline double TransformationSystemState::getInternalStateXd() const
{
  return internal_.getXd();
}
inline double TransformationSystemState::getInternalStateXdd() const
{
  return internal_.getXdd();
}
inline void TransformationSystemState::setTargetState(const State& target_state)
{
  target_ = target_state;
}
inline State TransformationSystemState::getTargetState() const
{
  return target_;
}
inline double TransformationSystemState::getTargetStateX() const
{
  return target_.getX();
}
inline double TransformationSystemState::getTargetStateXd() const
{
  return target_.getXd();
}
inline double TransformationSystemState::getTargetStateXdd() const
{
  return target_.getXdd();
}
inline void TransformationSystemState::setCurrentState(const State& current_state)
{
  current_ = current_state;
}
inline State TransformationSystemState::getCurrentState() const
{
  return current_;
}
inline double TransformationSystemState::getCurrentStateX() const
{
  return current_.getX();
}
inline double TransformationSystemState::getCurrentStateXd() const
{
  return current_.getXd();
}
inline double TransformationSystemState::getCurrentStateXdd() const
{
  return current_.getXdd();
}
inline void TransformationSystemState::setStart(const double start)
{
  start_ = start;
}
inline double TransformationSystemState::getStart() const
{
  return start_;
}
inline void TransformationSystemState::setGoal(const double goal)
{
  goal_ = goal;
}
inline double TransformationSystemState::getGoal() const
{
  return goal_;
}
inline void TransformationSystemState::setFT(const double ft)
{
  ft_ = ft;
}
inline double TransformationSystemState::getFT() const
{
  return ft_;
}
inline void TransformationSystemState::setF(const double f)
{
  f_ = f;
}
inline double TransformationSystemState::getF() const
{
  return f_;
}

}

#endif /* TRANSFORMATION_SYSTEM_STATE_BASE_H_ */
