/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_utilities.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 22, 2011

 *********************************************************************/

// system includes
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/file_io.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>
#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

#include <dynamic_movement_primitive/TypeMsg.h>

// local includes
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

using namespace std;
using namespace dmp;

namespace dmp_utilities
{

bool DynamicMovementPrimitiveUtilities::getControllerName(const dmp_lib::DMPPtr dmp,
                                                          std::string& controller_name)
{
  if (dmp->hasType(dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE))
  {
    controller_name.assign("/SL/" + dmp->getVersionString() + "DMPJointTrajectoryController");
  }
  else if (dmp->hasType(dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_SPACE)
      || dmp->hasType(dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE))
  {
    controller_name.assign("/SL/" + dmp->getVersionString() + "DMPCartesianTrajectoryController");
  }
  else
  {
    ROS_ERROR("DMP is of invalid type >%i<.", dmp->getType());
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveUtilities::getDMP(const dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg,
                                               dmp_lib::DMPPtr& dmp)
{
  if(dmp_utilities_msg.dmp_version == dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009)
  {
    return getDMP(dmp_utilities_msg.icra2009_dmp, dmp);
  }
  else if(dmp_utilities_msg.dmp_version == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
  {
    return getDMP(dmp_utilities_msg.nc2010_dmp, dmp);
  }
  else
  {
    ROS_ERROR("DMP version >%s< is invalid, needed to get DMP.", dmp_utilities_msg.dmp_version.c_str());
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveUtilities::getDMP(const dmp::ICRA2009DMPMsg& msg,
                                               dmp_lib::DMPPtr& dmp)
{
  ICRA2009DynamicMovementPrimitive::DMPPtr icra2009_dmp;
  if(!ICRA2009DynamicMovementPrimitive::createFromMessage(icra2009_dmp, msg))
  {
    return false;
  }
  dmp = icra2009_dmp;
  return true;
}

bool DynamicMovementPrimitiveUtilities::getDMP(const dmp::NC2010DMPMsg& msg,
                                               dmp_lib::DMPPtr& dmp)
{
  NC2010DynamicMovementPrimitive::DMPPtr nc2010_dmp;
  if(!NC2010DynamicMovementPrimitive::createFromMessage(nc2010_dmp, msg))
  {
    return false;
  }
  dmp = nc2010_dmp;
  return true;
}

bool DynamicMovementPrimitiveUtilities::writeToFile(const std::string& abs_bagfile_name, const dmp_lib::DMPPtr& dmp)
{
  if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009)
  {
    if(!ICRA2009DynamicMovementPrimitive::writeToDisc(boost::dynamic_pointer_cast<dmp_lib::ICRA2009DMP>(dmp), abs_bagfile_name))
    {
      return false;
    }
  }
  else if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
  {
    if(!NC2010DynamicMovementPrimitive::writeToDisc(boost::dynamic_pointer_cast<dmp_lib::NC2010DMP>(dmp), abs_bagfile_name))
    {
      return false;
    }
  }
  else
  {
    ROS_ERROR("DMP version >%s< is invalid, needed to get DMP.", dmp->getVersionString().c_str());
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveUtilities::readFromFile(const std::string& abs_bagfile_name, dmp_lib::DMPPtr& dmp)
{
  dmp_lib::ICRA2009DMPPtr icra2009_dmp;
  if(!DynamicMovementPrimitiveIO<ICRA2009DMP, ICRA2009DMPMsg>::readFromDisc(icra2009_dmp, abs_bagfile_name, false))
  {
    return false;
  }
  dmp_lib::NC2010DMPPtr nc2010_dmp;
  if(!DynamicMovementPrimitiveIO<NC2010DMP, NC2010DMPMsg>::readFromDisc(nc2010_dmp, abs_bagfile_name, false))
  {
    return false;
  }

  if(icra2009_dmp.get() && nc2010_dmp.get())
  {
    ROS_ERROR("Bag file >%s< contains more than just one DMP.", abs_bagfile_name.c_str());
    return false;
  }

  if(icra2009_dmp.get())
  {
    dmp = icra2009_dmp;
  }
  else if(nc2010_dmp.get())
  {
    dmp = nc2010_dmp;
  }
  else
  {
    ROS_ERROR("Bag file >%s< does not contain a DMP.", abs_bagfile_name.c_str());
    return false;
  }
  return true;
}

bool DynamicMovementPrimitiveUtilities::setMsg(const dmp_lib::DMPPtr& dmp,
                                               dmp::ICRA2009DMPMsg& msg)
{
  if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009)
  {
    return ICRA2009DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::ICRA2009DMP>(dmp), msg);
  }
  return false;
}

bool DynamicMovementPrimitiveUtilities::setMsg(const dmp_lib::DMPPtr& dmp,
                                               dmp::NC2010DMPMsg& msg)
{
  if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
  {
    return NC2010DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::NC2010DMP>(dmp), msg);
  }
  return false;
}

bool DynamicMovementPrimitiveUtilities::getGoal(const dmp::ICRA2009DMPMsg& msg,
                                                std::vector<double> goal)
{
  dmp_lib::DMPPtr dmp;
  if(!getDMP(msg, dmp))
  {
    return false;
  }
  return dmp->getGoal(goal, false);
}

bool DynamicMovementPrimitiveUtilities::getGoal(const dmp::NC2010DMPMsg& msg,
                                                std::vector<double> goal)
{
  dmp_lib::DMPPtr dmp;
  if(!getDMP(msg, dmp))
  {
    return false;
  }
  return dmp->getGoal(goal, false);
}

bool DynamicMovementPrimitiveUtilities::getGoal(const dmp::ICRA2009DMPMsg& msg,
                                                const std::vector<std::string>& variable_names,
                                                std::vector<double> goal)
{
  dmp_lib::DMPPtr dmp;
  if(!getDMP(msg, dmp))
  {
    return false;
  }
  return dmp->getGoal(variable_names, goal);
}

bool DynamicMovementPrimitiveUtilities::getGoal(const dmp::NC2010DMPMsg& msg,
                                                const std::vector<std::string>& variable_names,
                                                std::vector<double> goal)
{
  dmp_lib::DMPPtr dmp;
  if(!getDMP(msg, dmp))
  {
    return false;
  }
  return dmp->getGoal(variable_names, goal);
}

}
