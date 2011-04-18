/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file   skill_library_client.cpp

  \author	Peter Pastor
  \date		Mar 1, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <skill_library/getAffordance.h>
#include <skill_library/Affordance.h>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

// local includes
#include <skill_library/skill_library_client.h>

namespace skill_library
{

bool SkillLibraryClient::get(const std::string& dmp_name, dmp_lib::DMPPtr& dmp)
{
  // error checking
  if(dmp_name.empty())
  {
    ROS_ERROR("No DMP name is specified. Cannot get DMP from skill library.");
    return false;
  }

  bool service_online = false;
  while (ros::ok() && !service_online)
  {
    std::string service_name = "/SkillLibrary/getAffordance";
    get_affordance_service_client_ = node_handle_.serviceClient<skill_library::getAffordance> (service_name);
    if (!get_affordance_service_client_.waitForExistence(ros::Duration(1.0)))
    {
      ROS_WARN("Waiting for >%s< ...", service_name.c_str());
    }
    else
    {
      service_online = true;
    }
  }

  skill_library::Affordance affordance;
  affordance.object.name = dmp_name;

  getAffordance get_affordance_service;
  get_affordance_service.request.affordance = affordance;
  if (!get_affordance_service_client_.call(get_affordance_service))
  {
    ROS_ERROR("Could not retrieve DMP with name >%s< from the skill library.", dmp_name.c_str());
    return false;
  }
  if(get_affordance_service.response.result == get_affordance_service.response.FAILED)
  {
    ROS_ERROR("Retreiving DMP with name >%s< from the skill library failed.", dmp_name.c_str());
    return false;
  }
  affordance = get_affordance_service.response.affordance;

  affordance.dmp.dmp_version = dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009;
  if(!dmp_utilities::DynamicMovementPrimitiveUtilities::getDMP(affordance.dmp, dmp))
  {
    ROS_ERROR("Could not get DMP from message. Cannot get DMP from skill library.");
    return false;
  }

  return true;
}

}
