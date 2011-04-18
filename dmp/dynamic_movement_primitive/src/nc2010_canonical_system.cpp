/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		nc2010_canonical_system.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 9, 2010

 *********************************************************************/

// system includes
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <dmp_lib/nc2010_canonical_system_parameters.h>
#include <dmp_lib/nc2010_canonical_system_state.h>

// local includes
#include <dynamic_movement_primitive/nc2010_canonical_system.h>

namespace dmp
{

bool NC2010CanonicalSystem::initFromNodeHandle(dmp_lib::NC2010CSPtr& canonical_system,
                                                 ros::NodeHandle& node_handle)
{
  // create canonical system
  canonical_system.reset(new dmp_lib::NC2010CanonicalSystem());

  // create parameters
  dmp_lib::NC2010CSParamPtr parameters(new dmp_lib::NC2010CanonicalSystemParameters());
  // So far, nothing to be initialized

  // create state
  dmp_lib::NC2010CSStatePtr state(new dmp_lib::NC2010CanonicalSystemState());
  // So far, nothing to be initialized

  // initialize canonical system
  if(!canonical_system->initialize(parameters, state))
  {
    return false;
  }

  // initialize base class
  return CanonicalSystem::initFromNodeHandle(canonical_system, node_handle);
}

bool NC2010CanonicalSystem::initFromMessage(dmp_lib::NC2010CSPtr& canonical_system,
                                              const NC2010CSMsg& cs_msg)
{
  // create canonical system
  canonical_system.reset(new dmp_lib::NC2010CanonicalSystem());

  // create parameters
  dmp_lib::NC2010CSParamPtr parameters(new dmp_lib::NC2010CanonicalSystemParameters());
  // So far, nothing to be initialized

  // create state
  dmp_lib::NC2010CSStatePtr state(new dmp_lib::NC2010CanonicalSystemState());
  // So far, nothing to be initialized

  // initialize canonical system
  if(!canonical_system->initialize(parameters, state))
  {
    return false;
  }

  // initialize base class
  return CanonicalSystem::initFromMessage(canonical_system, cs_msg.canonical_system);
}

bool NC2010CanonicalSystem::writeToMessage(const dmp_lib::NC2010CSConstPtr canonical_system,
                                             NC2010CSMsg& cs_msg)
{
  // nc2010 parameters and state are empty

  // write base class
  return CanonicalSystem::writeToMessage(canonical_system, cs_msg.canonical_system);
}

}
