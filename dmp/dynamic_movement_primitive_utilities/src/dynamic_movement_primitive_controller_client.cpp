/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_controller_client.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 24, 2011

 *********************************************************************/

// system includes
#include <vector>
#include <string>

#include <usc_utilities/assert.h>
#include <robot_info/robot_info.h>
#include <dynamic_movement_primitive/TypeMsg.h>

// local includes
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

using namespace dmp;

namespace dmp_utilities
{

bool DynamicMovementPrimitiveControllerClient::initialize(// const std::string& robot_part_name,
                                                          // const std::string& base_frame_id,
                                                          const std::string& controller_name)
{
  std::vector<std::string> controller_names;
  controller_names.push_back(controller_name);
  std::string controller_namespace("");
  std::string robot_part_name = "null";
  return initialize(robot_part_name, controller_names, controller_namespace);
}

bool DynamicMovementPrimitiveControllerClient::initialize(const std::string& robot_part_name,
                                                          // const std::string& base_frame_id,
                                                          const std::vector<std::string>& controller_names,
                                                          const std::string& controller_namespace)
{
  ROS_VERIFY(!controller_names.empty());
  controller_names_ = controller_names;

  // robot_part_name_.assign(robot_part_name);
  // base_frame_id_.assign(base_frame_id);

  // robot_info::RobotInfo::RobotPart robot_part;
  // if(!robot_info::RobotInfo::getRobotPart(robot_part_name, robot_part))
  // {
  //   ROS_ERROR("Could not obtain robot part name >%s<. Cannot initialize DMP controller client.", robot_part_name.c_str());
  //   return false;
  // }

  // switch(robot_part)
  // {
  //   case robot_info::RobotInfo::RIGHT_ARM:
  //   {
      // TODO: Change the ICRA2009 default. Maybe make it a default parameter
      for (int i = 0; i < (int)controller_names.size(); ++i)
      {
        DMPControllerClientPtr icra2009_controller_client(new DynamicMovementPrimitiveControllerBaseClient<dmp::ICRA2009DMP, dmp::ICRA2009DMPMsg>());
        ROS_VERIFY(icra2009_controller_client->initialize(controller_namespace + controller_names[i]));
        icra2009_controller_clients_.insert(ControllerMapPair(controller_names[i], icra2009_controller_client));
      }
  //    break;
  //   }
  //   case robot_info::RobotInfo::LEFT_ARM:
  //   {
  //     // TODO: Change the ICRA2009 default. Maybe make it a default parameter
  //     for (int i = 0; i < (int)controller_names.size(); ++i)
  //     {
  //       DMPControllerClientPtr icra2009_controller_client(new DynamicMovementPrimitiveControllerBaseClient<dmp::ICRA2009DMP, dmp::ICRA2009DMPMsg>());
  //       ROS_VERIFY(icra2009_controller_client->initialize(controller_namespace + controller_names[i]));
  //       icra2009_controller_clients_.insert(ControllerMapPair(controller_names[i], icra2009_controller_client));
  //     }
  //     break;
  //   }
  //   case robot_info::RobotInfo::HEAD:
  //   {
  //     ROS_ERROR("Head dmp controller not implemented yet !!!");
  //     return false;
  //     break;
  //  }
  // }

  return (initialized_ = true);
}

void DynamicMovementPrimitiveControllerClient::setSingleThreadedMode(bool single_threaded_mode)
{
  ROS_ASSERT(initialized_);
  for(ControllerMapIterator mi = icra2009_controller_clients_.begin(); mi != icra2009_controller_clients_.end(); ++mi)
  {
    mi->second->setSingleThreadedMode(single_threaded_mode);
  }
}

bool DynamicMovementPrimitiveControllerClient::switchController(const std::string& controller_name)
{
  ROS_ASSERT(initialized_);
  ROS_INFO("Switching controller to >%s<", controller_name.c_str());

  if(isActive())
  {
    ROS_ERROR("Other DMP controller is still active, cannot switch to controller >%s<.", controller_name.c_str());
    return false;
  }

  current_controller_.assign(controller_name);
  return true;
}

bool DynamicMovementPrimitiveControllerClient::sendCommand(const dmp::ICRA2009DMPMsg& msg,
                                                           const std::string& controller_name,
                                                           bool wait_for_success)
{
  ROS_ASSERT(initialized_);

  dmp::ICRA2009DMPMsg dmp_msg = msg;
  if ((msg.dmp.parameters.type == dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE)
      || (msg.dmp.parameters.type == dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE)
      || (msg.dmp.parameters.type == dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE))
  {
    if (!switchController(controller_name))
    {
      return false;
    }
    ControllerMapIterator mi = icra2009_controller_clients_.find(controller_name);
    if(mi == icra2009_controller_clients_.end())
    {
      ROS_ERROR("There is no DMP controller with name >%s<. Controllers are:", controller_name.c_str());
      ControllerMapIterator it;
      for(it = icra2009_controller_clients_.begin(); it != icra2009_controller_clients_.end(); ++it)
      {
        ROS_ERROR(">%s<", it->first.c_str());
      }
      return false;
    }
    return mi->second->sendCommand(dmp_msg, wait_for_success);
  }
  else
  {
    ROS_ERROR("DMP message has invalid type >%i<. Cannot send it to the DMP controller.", msg.dmp.parameters.type);
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveControllerClient::sendCommand(const dmp::NC2010DMPMsg& msg,
                                                           const std::string& controller_name,
                                                           bool wait_for_success)
{
  ROS_ERROR("Sending NC2010 DMPs not implemented yet.");
  return false;
}

bool DynamicMovementPrimitiveControllerClient::sendCommand(const dmp_lib::DMPPtr& dmp,
                                                           const std::string& controller_name,
                                                           bool wait_for_success)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(dmp->isInitialized());

  if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009)
  {
    ICRA2009DynamicMovementPrimitive::DMPMsg dmp_msg;
    ICRA2009DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::ICRA2009DMP>(dmp), dmp_msg);
    return sendCommand(dmp_msg, controller_name, wait_for_success);
  }
  else if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
  {
    NC2010DynamicMovementPrimitive::DMPMsg dmp_msg;
    NC2010DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::NC2010DMP>(dmp), dmp_msg);
    return sendCommand(dmp_msg, controller_name, wait_for_success);
  }
  else
  {
    ROS_ERROR("Could not send DMP with of version >%s<.", dmp->getVersionString().c_str());
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveControllerClient::isActive()
{
  ROS_ASSERT(initialized_);
  for(ControllerMapIterator mi = icra2009_controller_clients_.begin(); mi != icra2009_controller_clients_.end(); ++mi)
  {
    if(mi->second->isActive())
    {
      ROS_WARN("DMP controller >%s< is active.", mi->first.c_str());
      return true;
    }
  }
  return false;
}

bool DynamicMovementPrimitiveControllerClient::getControllerStatus(dynamic_movement_primitive::ControllerStatusMsg& dmp_status)
{
  ROS_ASSERT(initialized_);
  ROS_INFO("Getting controller status of controller >%s<.", current_controller_.c_str());
  ControllerMapIterator mi = icra2009_controller_clients_.find(current_controller_);
  if(mi == icra2009_controller_clients_.end())
  {
    ROS_ERROR("There is not DMP controller with name >%s<.", current_controller_.c_str());
    return false;
  }
  return mi->second->getControllerStatus(dmp_status);
}

bool DynamicMovementPrimitiveControllerClient::waitForCompletion()
{
  ROS_ASSERT(initialized_);
  ControllerMapIterator mi = icra2009_controller_clients_.find(current_controller_);
  if(mi == icra2009_controller_clients_.end())
  {
    ROS_ERROR("There is not DMP controller with name >%s<.", current_controller_.c_str());
    return false;
  }
  return mi->second->waitForCompletion();
}

bool DynamicMovementPrimitiveControllerClient::halt(bool wait_for_success)
{
  ROS_ASSERT(initialized_);
  ControllerMapIterator mi = icra2009_controller_clients_.find(current_controller_);
  if(mi == icra2009_controller_clients_.end())
  {
    ROS_ERROR("There is not DMP controller with name >%s<.", current_controller_.c_str());
    return false;
  }
  return mi->second->halt(wait_for_success);
}

}
