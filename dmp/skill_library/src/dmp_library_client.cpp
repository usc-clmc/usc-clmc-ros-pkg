/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dmp_library_client.cpp

  \author	Peter Pastor
  \date		Jan 26, 2011

 *********************************************************************/

// system includes

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>
#include <dynamic_movement_primitive/DMPUtilitiesMsg.h>
// #include <usc_utilities/>

// local includes
#include <skill_library/dmp_library_client.h>

using namespace dmp;
using namespace std;

namespace skill_library
{

bool DMPLibraryClient::initialize(const string& library_root_directory)
{
  ROS_VERIFY(icra2009_dmp_library_.initialize(library_root_directory));
  // ROS_VERIFY(nc2010_dmp_library_.initialize(library_root_directory));
  return true;
}

bool DMPLibraryClient::reload()
{
  // todo: think about nc2010
  return icra2009_dmp_library_.reload();
}

bool DMPLibraryClient::print()
{
  // todo: think about nc2010
  return icra2009_dmp_library_.print();
}

bool DMPLibraryClient::printInfo(const std::string& description)
{
  // todo: think about nc2010
  return icra2009_dmp_library_.printInfo(description);
}

bool DMPLibraryClient::addDMP(dmp_lib::DMPPtr& dmp,
                              string& name)
{
  if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009)
  {
    ICRA2009DMP::DMPMsg msg;
    ICRA2009DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::ICRA2009DMP>(dmp), msg);
    ROS_VERIFY(addDMP(msg, name));
  }
  //  else if(dmp->getVersionString() == dynamic_movement_primitive::DMPUtilitiesMsg::NC2010)
  //  {
  //    NC2010DMP::DMPMsg msg;
  //    NC2010DynamicMovementPrimitive::writeToMessage(boost::dynamic_pointer_cast<dmp_lib::NC2010DMP>(dmp), msg);
  //    ROS_VERIFY(addDMP(msg, name));
  //  }
  else
  {
    ROS_ERROR("Could not send DMP with version >%s< to the controller.", dmp->getVersionString().c_str());
    return false;
  }
  return true;
}

// ICRA2009
bool DMPLibraryClient::addDMP(ICRA2009DMP::DMPMsg& msg,
                              string& name)
{
  return icra2009_dmp_library_.addDMP(msg, name);
}

bool DMPLibraryClient::getDMP(const string& name,
                              ICRA2009DMP::DMPMsg& msg)
{
  return icra2009_dmp_library_.getDMP(name, msg);
}

// NC2010
//bool DMPLibraryClient::addDMP(NC2010DMP::DMPMsg& msg,
//                              string& name)
//{
//  return nc2010_dmp_library_.addDMP(msg, name);
//}
//
//bool DMPLibraryClient::getDMP(const string& name,
//                              NC2010DMP::DMPMsg& msg)
//{
//  return nc2010_dmp_library_.getDMP(name, msg);
//}

}
