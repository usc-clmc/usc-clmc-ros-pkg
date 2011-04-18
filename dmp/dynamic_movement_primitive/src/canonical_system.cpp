/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		canonical_system.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 7, 2010

 *********************************************************************/

// system includes
#include <usc_utilities/param_server.h>

#include <dmp_lib/canonical_system_parameters.h>
#include <dmp_lib/canonical_system_state.h>

// local includes
#include <dynamic_movement_primitive/canonical_system.h>
#include <dynamic_movement_primitive/state.h>

namespace dmp
{

bool CanonicalSystem::initFromNodeHandle(dmp_lib::CSPtr canonical_system,
                                         ros::NodeHandle& node_handle)
{
    ros::NodeHandle cs_node_handle(node_handle, "canonical_system_parameters");

    // read parameters from canonical system node handle
    double alpha_x = 0;
    if(!usc_utilities::read(cs_node_handle, "alpha_x", alpha_x))
    {
      ROS_ERROR("Cannot read alpha_x from node handle in namespace >%s<.",node_handle.getNamespace().c_str());
      return false;
    }

    // initialize parameters
    if(!canonical_system->getParameters()->initialize(alpha_x))
    {
      return false;
    }
    return true;
}

bool CanonicalSystem::initFromMessage(dmp_lib::CSPtr canonical_system,
                                      const CSMsg& cs_msg)
{
  // initialize parameters
  if(!canonical_system->getParameters()->initialize(cs_msg.parameters.alpha_x))
  {
    return false;
  }

  State cs_state;
  if(!cs_state.initFromMessage(cs_msg.state.state))
  {
    return false;
  }

  canonical_system->getState()->set(cs_state, cs_msg.state.time);
  return true;
}

bool CanonicalSystem::writeToMessage(const dmp_lib::CSConstPtr canonical_system,
                                     CSMsg& cs_msg)
{
  dmp_lib::CSParamConstPtr parameters;
  dmp_lib::CSStateConstPtr state;
  if(!canonical_system->get(parameters, state))
  {
    return false;
  }

  // set parameters
  if(!parameters->get(cs_msg.parameters.alpha_x))
  {
    return false;
  }

  // set state
  dmp_lib::State current_state;
  state->get(current_state, cs_msg.state.time);
  current_state.get(cs_msg.state.state.x, cs_msg.state.state.xd, cs_msg.state.state.xdd);
  return true;
}

}

