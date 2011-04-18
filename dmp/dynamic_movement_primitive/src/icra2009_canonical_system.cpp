/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_canonical_system.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 9, 2010

 *********************************************************************/

// system includes
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <dmp_lib/icra2009_canonical_system_parameters.h>
#include <dmp_lib/icra2009_canonical_system_state.h>

// local includes
#include <dynamic_movement_primitive/icra2009_canonical_system.h>

namespace dmp
{

bool ICRA2009CanonicalSystem::initFromNodeHandle(dmp_lib::ICRA2009CSPtr& canonical_system,
                                                 ros::NodeHandle& node_handle)
{
  // create canonical system
  canonical_system.reset(new dmp_lib::ICRA2009CanonicalSystem());

  // create parameters
  dmp_lib::ICRA2009CSParamPtr parameters(new dmp_lib::ICRA2009CanonicalSystemParameters());
  // So far, nothing to be initialized

  // create state
  dmp_lib::ICRA2009CSStatePtr state(new dmp_lib::ICRA2009CanonicalSystemState());
  // So far, nothing to be initialized

  // initialize canonical system
  if(!canonical_system->initialize(parameters, state))
  {
    return false;
  }

  // initialize base class
  return CanonicalSystem::initFromNodeHandle(canonical_system, node_handle);
}

bool ICRA2009CanonicalSystem::initFromMessage(dmp_lib::ICRA2009CSPtr& canonical_system,
                                              const ICRA2009CSMsg& cs_msg)
{
  // create canonical system
  canonical_system.reset(new dmp_lib::ICRA2009CanonicalSystem());

  // create parameters
  dmp_lib::ICRA2009CSParamPtr parameters(new dmp_lib::ICRA2009CanonicalSystemParameters());
  // So far, nothing to be initialized

  // create state
  dmp_lib::ICRA2009CSStatePtr state(new dmp_lib::ICRA2009CanonicalSystemState());
  // So far, nothing to be initialized

  // initialize canonical system
  if(!canonical_system->initialize(parameters, state))
  {
    return false;
  }

  // initialize base class
  return CanonicalSystem::initFromMessage(canonical_system, cs_msg.canonical_system);
}

bool ICRA2009CanonicalSystem::writeToMessage(const dmp_lib::ICRA2009CSConstPtr canonical_system,
                                             ICRA2009CSMsg& cs_msg)
{
  // icra2009 parameters and state are empty

  // write base class
  return CanonicalSystem::writeToMessage(canonical_system, cs_msg.canonical_system);
}

}
