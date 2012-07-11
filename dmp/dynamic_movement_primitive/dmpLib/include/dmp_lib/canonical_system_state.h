/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		canonical_system_state.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 7, 2010

 *********************************************************************/

#ifndef CANONICAL_SYSTEM_STATE_BASE_H_
#define CANONICAL_SYSTEM_STATE_BASE_H_

// system includes
#include <boost/shared_ptr.hpp>
#include <math.h>

// local includes
#include <dmp_lib/state.h>

namespace dmp_lib
{

class CanonicalSystemState
{

  /*! Allow the CanonicalSystem class and TransformationSystem class to access private member variables directly
   */
  friend class CanonicalSystem;
  friend class TransformationSystem;

public:

  /*! Constructor
   */
  CanonicalSystemState() :
    time_(0), progress_time_(0) {};

  /*! Destructor
   */
  virtual ~CanonicalSystemState() {};

  /*! Only check whether the state is the same
    * @param state
    * @return True if equal, otherwise False
    */
  bool operator==(const CanonicalSystemState &state) const
  {
    return (state_ == state.state_) && (fabs(time_ - state.time_) < EQUALITY_PRECISSION);
  }
  bool operator!=(const CanonicalSystemState &state) const
  {
    return !(*this == state);
  }

  /*!
   * @param other_state
   * @return
   */
  bool isCompatible(const CanonicalSystemState& other_state) const;

  /*!
   * @param state
   * @param time
   */
  void set(const State& state,
           const double time);

  /*!
   * @param state
   * @param time
   */
  void get(State& state,
           double& time) const;

  /*!
   * @return
   */
  // State getState() const;

  /*!
   * @return
   */
  double getStateX() const;

  /*!
   * @return
   */
  double getStateXd() const;

  /*!
   * @return
   */
  double getCanX() const;

  /*!
   * @return
   */
  void setCanX(const double x);

  /*!
   * @return
   */
  void setState(const State& state);

  /*!
   * @param x
   */
  void setStateX(const double x);

  /*!
   * @param xd
   */
  void setStateXd(const double xd);

  /*!
   * @return
   */
  double getTime() const;

  /*!
   * @param time
   */
  void setTime(const double time);

  /*!
   * @param dt
   */
  void addTime(const double dt);

  /*!
   * @return
   */
  double getProgressTime() const;

  /*!
   * @return
   */
  void setProgressTime(const double time);

  /*!
   * @param dt
   */
  void addProgressTime(const double dt);



protected:

  static const double EQUALITY_PRECISSION = 1e-6;

  /*!
   */
  State state_;
  double time_;
  double progress_time_;

};

/*! Abbreviation for convinience
 */
typedef CanonicalSystemState CSState;
typedef boost::shared_ptr<CSState> CSStatePtr;
typedef boost::shared_ptr<CSState const> CSStateConstPtr;

inline void CanonicalSystemState::set(const State& state,
                                      const double time)
{
  state_ = state;
  time_ = time;
}
inline void CanonicalSystemState::get(State& state,
                                      double& time) const
{
  state = state_;
  time = time_;
}

inline bool CanonicalSystemState::isCompatible(const CanonicalSystemState& /*other_state*/) const
{
  // for now canonical system states are always compatible.
  return true;
}

// inline State CanonicalSystemState::getState() const
// {
//   return state_;
// }
inline double CanonicalSystemState::getStateX() const
{
  return state_.getX();
}
inline double CanonicalSystemState::getCanX() const
{
  // small hacks here and there keep things funny (TODO: change this)
  return state_.getXdd();
}
inline double CanonicalSystemState::getStateXd() const
{
  return state_.getXd();
}
inline void CanonicalSystemState::setState(const State& state)
{
  state_ = state;
}
inline void CanonicalSystemState::setStateX(const double x)
{
  state_.setX(x);
}
inline void CanonicalSystemState::setStateXd(const double xd)
{
  state_.setX(xd);
}
inline void CanonicalSystemState::setCanX(const double x)
{
  // small hacks here and there keep things funny (TODO: change this)
  state_.setXdd(x);
}
inline double CanonicalSystemState::getTime() const
{
  return time_;
}
inline void CanonicalSystemState::setTime(const double time)
{
  time_ = time;
}
inline void CanonicalSystemState::addTime(const double dt)
{
  time_ += dt;
}
inline double CanonicalSystemState::getProgressTime() const
{
  return progress_time_;
}
inline void CanonicalSystemState::setProgressTime(const double time)
{
  progress_time_ = time;
}
inline void CanonicalSystemState::addProgressTime(const double dt)
{
  progress_time_ += dt;
}

}

#endif /* CANONICAL_SYSTEM_STATE_BASE_H_ */
