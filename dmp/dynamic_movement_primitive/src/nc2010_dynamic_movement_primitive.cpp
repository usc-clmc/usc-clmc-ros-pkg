/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		nc2010_dynamic_movement_primitive.cpp

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
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_transformation_system.h>
#include <dynamic_movement_primitive/nc2010_canonical_system.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>

using namespace std;

namespace dmp
{

bool NC2010DynamicMovementPrimitive::initFromNodeHandle(DMPPtr& dmp,
                                                          const std::vector<std::string>& robot_part_names,
                                                          ros::NodeHandle& node_handle)
{
  // allocate memory
  dmp.reset(new dmp_lib::NC2010DynamicMovementPrimitive());

  // create nc2010 dmp parameters
  dmp_lib::NC2010DMPParamPtr parameters(new dmp_lib::NC2010DynamicMovementPrimitiveParameters());
  // no initialization required... will initialize parameters from node handle once there are additional nc2010 parameters

  // create nc2010 dmp state
  dmp_lib::NC2010DMPStatePtr state(new dmp_lib::NC2010DynamicMovementPrimitiveState());
  // no initialization required... will initialize parameters from node handle once there are additional nc2010 state parameters

  // create transformation systems
  vector<dmp_lib::NC2010TSPtr> transformation_systems;
  ROS_VERIFY(NC2010TransformationSystem::initFromNodeHandle(transformation_systems, robot_part_names, node_handle));

  // create canonical system
  dmp_lib::NC2010CSPtr canonical_system;
  ROS_VERIFY(NC2010CanonicalSystem::initFromNodeHandle(canonical_system, node_handle));

  // finally create dmp using all that
  ROS_VERIFY(dmp->initialize(parameters, state, transformation_systems, canonical_system));

  // initialize base class
  ROS_VERIFY(DynamicMovementPrimitive::initFromNodeHandle(dmp, node_handle));
  return true;
}

bool NC2010DynamicMovementPrimitive::createFromMessage(DMPPtr& dmp,
                                                         const DMPMsg& dmp_msg)
{
  // allocate memory and initialize it from message
  dmp.reset(new dmp_lib::NC2010DynamicMovementPrimitive());
  return NC2010DynamicMovementPrimitive::initFromMessage(dmp, dmp_msg);
}

// used within non-real-time part of the dmp controller. Therefore avoid assertions
bool NC2010DynamicMovementPrimitive::initFromMessage(const DMPPtr dmp,
                                                       const DMPMsg& dmp_msg)
{
  if(dmp.get() == NULL)
  {
    ROS_ERROR("Cannot initialize >%s< DMP message from empty pointer.", NC2010DynamicMovementPrimitive::getVersionString().c_str());
    return false;
  }

  // create nc2010 dmp parameters
  dmp_lib::NC2010DMPParamPtr parameters(new dmp_lib::NC2010DynamicMovementPrimitiveParameters());
  // no initialization required... will initialize parameters from message once there are additional nc2010 parameters

  // create nc2010 dmp state
  dmp_lib::NC2010DMPStatePtr state(new dmp_lib::NC2010DynamicMovementPrimitiveState());
  // no initialization required... will initialize parameters from message once there are additional nc2010 state parameters

  // create transformation systems
  vector<dmp_lib::NC2010TSPtr> transformation_systems;
  if(!NC2010TransformationSystem::initFromMessage(transformation_systems, dmp_msg.transformation_systems))
  {
    return false;
  }

  // create canonical system
  dmp_lib::NC2010CSPtr canonical_system;
  if(!NC2010CanonicalSystem::initFromMessage(canonical_system, dmp_msg.canonical_system))
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

bool NC2010DynamicMovementPrimitive::writeToMessage(const DMPConstPtr dmp,
                                                      DMPMsg& nc2010_dmp_msg)
{
  // write base class to message
  ROS_VERIFY(DynamicMovementPrimitive::writeToMessage(dmp, nc2010_dmp_msg.dmp));

  dmp_lib::NC2010DMPParamConstPtr parameters;
  dmp_lib::NC2010DMPStateConstPtr state;
  std::vector<dmp_lib::NC2010TSConstPtr> transformation_systems;
  dmp_lib::NC2010CSConstPtr canonical_system;
  ROS_VERIFY(dmp->get(parameters, state, transformation_systems, canonical_system));

  // nc2010 dmp parameters and state are empty, not writing anything.

  // write transformation systems to message

  ROS_VERIFY(NC2010TransformationSystem::writeToMessage(transformation_systems, nc2010_dmp_msg.transformation_systems));

  // write canonical systems to message
  ROS_VERIFY(NC2010CanonicalSystem::writeToMessage(canonical_system, nc2010_dmp_msg.canonical_system));
  return true;
}

bool NC2010DynamicMovementPrimitive::writeToDisc(const DMPConstPtr dmp,
                                                   const std::string& abs_bag_file_name)
{
  DMPMsg nc2010_dmp_msg;
  ROS_VERIFY(writeToMessage(dmp, nc2010_dmp_msg));
  return DynamicMovementPrimitiveIO<NC2010DMP, DMPMsg>::writeToDisc(nc2010_dmp_msg, abs_bag_file_name);
}

bool NC2010DynamicMovementPrimitive::readFromDisc(DMPPtr& dmp,
                                                    const std::string& abs_bag_file_name)
{
  return DynamicMovementPrimitiveIO<NC2010DMP, DMPMsg>::readFromDisc(dmp, abs_bag_file_name);
}

}

