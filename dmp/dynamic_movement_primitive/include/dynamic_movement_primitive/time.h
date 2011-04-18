/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		time.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 3, 2011

 *********************************************************************/

#ifndef TIME_H_
#define TIME_H_

// system includes
#include <ros/ros.h>
#include <dmp_lib/time.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <dynamic_movement_primitive/TimeMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::TimeMsg TimeMsg;


class Time : public dmp_lib::Time
{

public:

  /*! Constructor
   */
  Time() {};
  Time(const TimeMsg& time) :
    dmp_lib::Time(time.delta_t, time.tau) {};

  /*! Destructor
   */
  virtual ~Time() {};

  /*!
   * @param time
   * @return True if initialization is successful, otherwise False
   */
  bool initFromMessage(const TimeMsg& time);

  /*!
   * @param time
   * @return True if initialization is successful, otherwise False
   */
  bool writeToMessage(TimeMsg& time) const;

  /*!
   * @return
   */
  TimeMsg toMessage() const;

  /*!
   * @param node_handle
   * @return True if initialization is successful, otherwise False
   */
  bool initFromNodeHandle(ros::NodeHandle& node_handle);

  /*!
   * @return
   */
  const dmp_lib::Time getTime() const;

private:

};

// Inline functions follow
inline bool Time::initFromMessage(const TimeMsg& time)
{
  if (!setTau(time.tau))
  {
    return false;
  }
  if (!setDeltaT(time.delta_t))
  {
    return false;
  }
  return true;
}

inline bool Time::writeToMessage(TimeMsg& time) const
{
  time.tau = getTau();
  time.delta_t = getDeltaT();
  return true;
}

inline TimeMsg Time::toMessage() const
{
  TimeMsg time;
  ROS_VERIFY(writeToMessage(time));
  return time;
}

inline bool Time::initFromNodeHandle(ros::NodeHandle& node_handle)
{
  double tau = 0;
  if(!usc_utilities::read(node_handle, "tau", tau))
  {
    ROS_ERROR("Could not read parameter >tau< from param server in namespace >%s<", node_handle.getNamespace().c_str());
    return false;
  }
  ROS_VERIFY(setTau(tau));

  double delta_t = 0;
  if(!usc_utilities::read(node_handle, "delta_t", delta_t))
  {
    ROS_ERROR("Could not read parameter >delta_t< from param server in namespace >%s<", node_handle.getNamespace().c_str());
    return false;
  }
  ROS_VERIFY(setDeltaT(delta_t));
  return true;
}

inline const dmp_lib::Time Time::getTime() const
{
  return dmp_lib::Time(getDeltaT(), getTau());
}

}

#endif /* TIME_H_ */
