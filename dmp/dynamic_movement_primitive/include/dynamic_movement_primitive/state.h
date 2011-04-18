/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		state.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 10, 2010

 *********************************************************************/

#ifndef STATE_H_
#define STATE_H_

// system includes
#include <ros/ros.h>
#include <dmp_lib/state.h>

// local includes
#include <dynamic_movement_primitive/StateMsg.h>

namespace dmp
{

class State : public dmp_lib::State
{

  /*! Abbreviation for convinience
   */
  typedef dynamic_movement_primitive::StateMsg StateMsg;

public:

  /*! Constructor
   */
  State() {};
//  State(const double x, const double xd, const double xdd) :
  //      dmp_lib::State(x, xd, xdd) {};
  State(const StateMsg& state) :
    dmp_lib::State(state.x, state.xd, state.xdd) {};

  /*! Destructor
   */
  virtual ~State() {};

  /*!
   * @param state
   * @return True if initialization is successful, otherwise False
   */
  bool initFromMessage(const StateMsg& state);

  /*!
   * @param state
   * @return True if initialization is successful, otherwise False
   */
  bool writeToMessage(StateMsg& state);

  /*!
   * @param node_handle
   * @return True if initialization is successful, otherwise False
   */
  bool initFromNodeHandle(ros::NodeHandle& node_handle);

  /*!
   * @return
   */
  const dmp_lib::State getState() const;

private:

};

// Inline functions follow
inline bool State::initFromMessage(const StateMsg& state)
{
  set(state.x, state.xd, state.xdd);
  return true;
}
inline bool State::writeToMessage(StateMsg& state)
{
  get(state.x, state.xd, state.xdd);
  return true;
}
inline const dmp_lib::State State::getState() const
{
  return dmp_lib::State(x_, xd_, xdd_);
}

}

#endif /* STATE_H_ */
