/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		services.h

  \author	Peter Pastor
  \date		Jul 30, 2011

 *********************************************************************/

#ifndef SERVICES_H_
#define SERVICES_H_

// system includes
#include <ros/ros.h>

// local includes

namespace usc_utilities
{

void waitFor(ros::ServiceClient& service_client, ros::Duration time_out = ros::Duration(0.3));
bool isReady(ros::ServiceClient& service_client, ros::Duration time_out = ros::Duration(0.3));

// inline functions follow

inline void waitFor(ros::ServiceClient& service_client, ros::Duration time_out)
{
  bool service_online = false;
  while (ros::ok() && !service_online)
  {
    if (isReady(service_client, time_out))
    {
      service_online = true;
    }
    else
    {
      ROS_WARN("Waiting for service >%s< ...", service_client.getService().c_str());
    }
  }
}

inline bool isReady(ros::ServiceClient& service_client, ros::Duration time_out)
{
  return service_client.waitForExistence(time_out);
}

}

#endif /* SERVICES_H_ */
