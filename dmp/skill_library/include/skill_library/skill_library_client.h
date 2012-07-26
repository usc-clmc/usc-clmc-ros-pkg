/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file   skill_library_client.h

  \author	Peter Pastor
  \date		Mar 1, 2011

 *********************************************************************/

#ifndef SKILL_LIBRARY_CLIENT_H_
#define SKILL_LIBRARY_CLIENT_H_

// system includes
#include <ros/ros.h>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h>

// local includes

namespace skill_library
{

class SkillLibraryClient
{

public:

  /*! Constructor
   * @param node_handle
   * @param wait_for_service
   */
  SkillLibraryClient(ros::NodeHandle node_handle);

  /*! Destructor
   */
  virtual ~SkillLibraryClient() {};

  /*!
   * @param dmp_name Name must be of the form <description>
   * @param id
   * @param dmp
   * @return True on success, otherwise False
   */
  bool get(const std::string& dmp_name, const int id, dmp_lib::DMPPtr& dmp);

  /*!
   * @param dmp_name Name must be of the form <description_id>
   * @param dmp
   * @return True on success, otherwise False
   */
  bool get(const std::string& dmp_name, dmp_lib::DMPPtr& dmp);

private:
  ros::NodeHandle node_handle_;
  ros::ServiceClient get_affordance_service_client_;

};

}

#endif /* SKILL_LIBRARY_CLIENT_H_ */
