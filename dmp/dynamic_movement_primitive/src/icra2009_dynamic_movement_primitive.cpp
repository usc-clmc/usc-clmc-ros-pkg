/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		icra2009_dynamic_movement_primitive.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 3, 2010

 *********************************************************************/

// system includes
#include <boost/foreach.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

// local includes
#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/icra2009_transformation_system.h>
#include <dynamic_movement_primitive/icra2009_canonical_system.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>

using namespace std;

namespace dmp
{

bool ICRA2009DynamicMovementPrimitive::initFromNodeHandle(DMPPtr& dmp,
                                                          ros::NodeHandle& node_handle)
{
  std::vector<std::string> empty_robot_part_names;
  return ICRA2009DynamicMovementPrimitive::initFromNodeHandle(dmp, empty_robot_part_names, node_handle);
}

bool ICRA2009DynamicMovementPrimitive::initFromNodeHandle(DMPPtr& dmp,
                                                          const std::vector<std::string>& robot_part_names,
                                                          ros::NodeHandle& node_handle)
{
  ROS_DEBUG("Initializing ICRA2009 DMP.");

  // allocate memory
  dmp.reset(new dmp_lib::ICRA2009DynamicMovementPrimitive());

  // create icra2009 dmp parameters
  dmp_lib::ICRA2009DMPParamPtr parameters(new dmp_lib::ICRA2009DynamicMovementPrimitiveParameters());
  // no initialization required... will initialize parameters from node handle once there are additional icra2009 parameters

  // create icra2009 dmp state
  dmp_lib::ICRA2009DMPStatePtr state(new dmp_lib::ICRA2009DynamicMovementPrimitiveState());
  // no initialization required... will initialize parameters from node handle once there are additional icra2009 state parameters

  // create transformation systems
  vector<dmp_lib::ICRA2009TSPtr> transformation_systems;
  ROS_VERIFY(ICRA2009TransformationSystem::initFromNodeHandle(transformation_systems, robot_part_names, node_handle));

  // create canonical system
  dmp_lib::ICRA2009CSPtr canonical_system;
  ROS_VERIFY(ICRA2009CanonicalSystem::initFromNodeHandle(canonical_system, node_handle));

  // finally create dmp using all that
  ROS_VERIFY(dmp->initialize(parameters, state, transformation_systems, canonical_system));

  // initialize base class
  ROS_VERIFY(DynamicMovementPrimitive::initFromNodeHandle(dmp, node_handle));

  ROS_DEBUG("Done initializing ICRA2009 DMP.");
  return true;
}

bool ICRA2009DynamicMovementPrimitive::createFromMessage(DMPPtr& dmp,
                                                         const DMPMsg& dmp_msg)
{
  // allocate memory and initialize it from message
  dmp.reset(new dmp_lib::ICRA2009DynamicMovementPrimitive());
  return ICRA2009DynamicMovementPrimitive::initFromMessage(dmp, dmp_msg);
}

// used within non-real-time part of the dmp controller. Therefore avoid assertions
bool ICRA2009DynamicMovementPrimitive::initFromMessage(const DMPPtr dmp,
                                                       const DMPMsg& dmp_msg)
{
  if(dmp.get() == NULL)
  {
    ROS_ERROR("Cannot initialize >%s< DMP message from empty pointer.", ICRA2009DynamicMovementPrimitive::getVersionString().c_str());
    return false;
  }

  // create icra2009 dmp parameters
  dmp_lib::ICRA2009DMPParamPtr parameters(new dmp_lib::ICRA2009DynamicMovementPrimitiveParameters());
  // no initialization required... will initialize parameters from message once there are additional icra2009 parameters

  // create icra2009 dmp state
  dmp_lib::ICRA2009DMPStatePtr state(new dmp_lib::ICRA2009DynamicMovementPrimitiveState());
  // no initialization required... will initialize parameters from message once there are additional icra2009 state parameters

  // create transformation systems
  vector<dmp_lib::ICRA2009TSPtr> transformation_systems;
  if(!ICRA2009TransformationSystem::initFromMessage(transformation_systems, dmp_msg.transformation_systems))
  {
    return false;
  }

  // create canonical system
  dmp_lib::ICRA2009CSPtr canonical_system;
  if(!ICRA2009CanonicalSystem::initFromMessage(canonical_system, dmp_msg.canonical_system))
  {
    return false;
  }

  // finally create dmp using all that
  if(!dmp->initialize(parameters, state, transformation_systems, canonical_system))
  {
    return false;
  }

  // initialize base class
  return DynamicMovementPrimitive::initFromMessage(dmp, dmp_msg.dmp);
}

bool ICRA2009DynamicMovementPrimitive::writeToMessage(const DMPConstPtr dmp,
                                                      DMPMsg& icra2009_dmp_msg)
{
  // write base class to message
  ROS_VERIFY(DynamicMovementPrimitive::writeToMessage(dmp, icra2009_dmp_msg.dmp));

  dmp_lib::ICRA2009DMPParamConstPtr parameters;
  dmp_lib::ICRA2009DMPStateConstPtr state;
  std::vector<dmp_lib::ICRA2009TSConstPtr> transformation_systems;
  dmp_lib::ICRA2009CSConstPtr canonical_system;
  ROS_VERIFY(dmp->get(parameters, state, transformation_systems, canonical_system));

  // icra2009 dmp parameters and state are empty, not writing anything.
  // write transformation systems to message

  ROS_VERIFY(ICRA2009TransformationSystem::writeToMessage(transformation_systems, icra2009_dmp_msg.transformation_systems));

  // write canonical systems to message
  ROS_VERIFY(ICRA2009CanonicalSystem::writeToMessage(canonical_system, icra2009_dmp_msg.canonical_system));
  return true;
}

bool ICRA2009DynamicMovementPrimitive::writeToDisc(const DMPConstPtr dmp,
                                                   const std::string& abs_bag_file_name)
{
  DMPMsg icra2009_dmp_msg;
  ROS_VERIFY(writeToMessage(dmp, icra2009_dmp_msg));
  return DynamicMovementPrimitiveIO<ICRA2009DMP, DMPMsg>::writeToDisc(icra2009_dmp_msg, abs_bag_file_name);
}

bool ICRA2009DynamicMovementPrimitive::readFromDisc(DMPPtr& dmp,
                                                    const std::string& abs_bag_file_name)
{
  return DynamicMovementPrimitiveIO<ICRA2009DMP, DMPMsg>::readFromDisc(dmp, abs_bag_file_name);
}

}

