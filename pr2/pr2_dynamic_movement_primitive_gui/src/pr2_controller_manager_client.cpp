/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_controller_manager_client.cpp

  \author	Peter Pastor
  \date		Jul 30, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/services.h>

#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/ReloadControllerLibraries.h>

// local includes
#include <pr2_dynamic_movement_primitive_gui/pr2_controller_manager_client.h>

namespace pr2_dynamic_movement_primitive_gui
{

PR2ControllerManagerClient::PR2ControllerManagerClient(ros::NodeHandle node_handle) :
    node_handle_(node_handle)
{
  list_controllers_service_client_ = node_handle_.serviceClient<pr2_mechanism_msgs::ListControllers> (std::string("/pr2_controller_manager/list_controllers"));
  usc_utilities::waitFor(list_controllers_service_client_);
  switch_controller_service_client_ = node_handle_.serviceClient<pr2_mechanism_msgs::SwitchController> (std::string("/pr2_controller_manager/switch_controller"));
  usc_utilities::waitFor(switch_controller_service_client_);
  load_controller_service_client_ = node_handle_.serviceClient<pr2_mechanism_msgs::LoadController> (std::string("/pr2_controller_manager/load_controller"));
  usc_utilities::waitFor(load_controller_service_client_);
  unload_controller_service_client_ = node_handle_.serviceClient<pr2_mechanism_msgs::UnloadController> (std::string("/pr2_controller_manager/unload_controller"));
  usc_utilities::waitFor(unload_controller_service_client_);
  reload_libraries_controllers_service_client_ = node_handle_.serviceClient<pr2_mechanism_msgs::ReloadControllerLibraries> (std::string("/pr2_controller_manager/reload_controller_libraries"));
  usc_utilities::waitFor(reload_libraries_controllers_service_client_);

  ROS_VERIFY(readParams());
}

bool PR2ControllerManagerClient::readParams()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, "right_arm_dmp_ik_controller_name", right_arm_dmp_ik_controller_name_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "left_arm_dmp_ik_controller_name", left_arm_dmp_ik_controller_name_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "dual_arm_dmp_ik_controller_name", dual_arm_dmp_ik_controller_name_));
  return true;
}

bool PR2ControllerManagerClient::stopController(const std::string& controller_name)
{
  ROS_INFO("Stopping controller >%s<.", controller_name.c_str());
  std::vector<std::string> start_controller_names;
  std::vector<std::string> stop_controller_names;
  stop_controller_names.push_back(controller_name);
  return switchControllers(start_controller_names, stop_controller_names);
}

bool PR2ControllerManagerClient::startController(const std::string& controller_name)
{
  ROS_INFO("Starting controller >%s<.", controller_name.c_str());
  std::vector<std::string> start_controller_names;
  start_controller_names.push_back(controller_name);
  std::vector<std::string> stop_controller_names;
  return switchControllers(start_controller_names, stop_controller_names);
}

bool PR2ControllerManagerClient::switchController(const std::string& start_controller_name,
                                                  const std::string& stop_controller_name)
{
  std::vector<std::string> start_controller_names;
  start_controller_names.push_back(start_controller_name);
  std::vector<std::string> stop_controller_names;
  stop_controller_names.push_back(stop_controller_name);
  return switchControllers(start_controller_names, stop_controller_names);
}

bool PR2ControllerManagerClient::switchControllers(const std::vector<std::string>& start_controller_names,
                                                   const std::vector<std::string>& stop_controller_names)
{
  pr2_mechanism_msgs::SwitchController::Request request;
  pr2_mechanism_msgs::SwitchController::Response response;
  request.start_controllers = start_controller_names;
  request.stop_controllers = stop_controller_names;
  request.strictness = pr2_mechanism_msgs::SwitchController::Request::STRICT;
  ROS_VERIFY(switch_controller_service_client_.call(request, response));
  return response.ok;
}

bool PR2ControllerManagerClient::loadController(const std::string controller_name)
{
  ROS_INFO("Loading controller >%s<.", controller_name.c_str());
  pr2_mechanism_msgs::LoadController::Request request;
  pr2_mechanism_msgs::LoadController::Response response;
  request.name = controller_name;
  ROS_VERIFY(load_controller_service_client_.call(request, response));
  return response.ok;
}

bool PR2ControllerManagerClient::unloadController(const std::string controller_name)
{
  ROS_INFO("Unloading controller >%s<.", controller_name.c_str());
  pr2_mechanism_msgs::UnloadController::Request request;
  pr2_mechanism_msgs::UnloadController::Response response;
  request.name = controller_name;
  ROS_VERIFY(unload_controller_service_client_.call(request, response));
  return response.ok;
}

bool PR2ControllerManagerClient::getControllerList(std::vector<std::string>& controller_names,
                                                   std::vector<ControllerState>& controller_states)
{
  pr2_mechanism_msgs::ListControllers::Request request;
  pr2_mechanism_msgs::ListControllers::Response response;
  ROS_VERIFY(list_controllers_service_client_.call(request, response));

  controller_names.clear();
  controller_states.clear();
  controller_names = response.controllers;
  for (int i = 0; i < (int)response.state.size(); ++i)
  {
    if(response.state[i].compare("running") == 0)
    {
      controller_states.push_back(RUNNING);
    }
    else if(response.state[i].compare("stopped") == 0)
    {
      controller_states.push_back(STOPPED);
    }
    else
    {
      ROS_ERROR("Unknown state received from >%s<.", list_controllers_service_client_.getService().c_str());
      return false;
    }
  }
  ROS_ASSERT_MSG(controller_names.size() == controller_states.size(), "Number of controllers >%i< does not correspond to number of states >%i<. This should never happen.",
                 (int)controller_names.size(), (int)controller_states.size());
  return true;
}

bool PR2ControllerManagerClient::reloadControllers()
{
  std::vector<std::string> controller_names;
  std::vector<ControllerState> controller_states;
  ROS_VERIFY(getControllerList(controller_names, controller_states));

  for(int i=0; i<(int)controller_names.size(); ++i)
  {
    if(controller_states[i] == RUNNING)
    {
      stopController(controller_names[i]);
    }
    unloadController(controller_names[i]);
  }

  pr2_mechanism_msgs::ReloadControllerLibraries::Request request;
  request.force_kill = false;
  pr2_mechanism_msgs::ReloadControllerLibraries::Response response;
  ROS_VERIFY(reload_libraries_controllers_service_client_.call(request, response));

  if(!response.ok)
  {
    ROS_ERROR("Problems when reloading controller libraries.");
    return false;
  }

  for (int i = 0; i < (int)controller_names.size(); ++i)
  {
    loadController(controller_names[i]);
    if(controller_states[i] == RUNNING)
    {
      startController(controller_names[i]);
    }
  }

  return true;
}


}
