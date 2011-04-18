/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		behavior_utilities.h

  \author	Peter Pastor
  \date		Jan 26, 2011

 *********************************************************************/

#ifndef BEHAVIOR_UTILITIES_H_
#define BEHAVIOR_UTILITIES_H_

// system includes
#include <string>
#include <ros/ros.h>

// local includes

namespace dmp_behaviors
{

template<class Behavior, class ActionServer>
  class BehaviorUtilities
  {

  public:

  static void failed(const std::string& reason,
                     ActionServer& action_server);

  private:
    BehaviorUtilities() {};
    virtual ~BehaviorUtilities() {};

  };

template<class Behavior, class ActionServer>
  void BehaviorUtilities<Behavior, ActionServer>::failed(const std::string& reason,
                                                         ActionServer& action_server)
  {
    ROS_ERROR("%s", reason.c_str());
    typename Behavior::ActionResult result;
    result.result = Behavior::ActionResult::FAILED;
    action_server.setSucceeded(result);
  }

}

#endif /* BEHAVIOR_UTILITIES_H_ */
