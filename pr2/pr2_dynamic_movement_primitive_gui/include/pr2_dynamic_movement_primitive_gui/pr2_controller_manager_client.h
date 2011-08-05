/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_controller_manager_client.h

  \author	Peter Pastor
  \date		Jul 30, 2011

 *********************************************************************/

#ifndef PR2_CONTROLLER_MANAGER_CLIENT_H_
#define PR2_CONTROLLER_MANAGER_CLIENT_H_

// system includes
#include <ros/ros.h>
#include <vector>
#include <string>

// local includes

namespace pr2_dynamic_movement_primitive_gui
{

class PR2ControllerManagerClient
{

public:

  /*! Constructor
   */
  PR2ControllerManagerClient(ros::NodeHandle);
  /*! Destructor
   */
  virtual ~PR2ControllerManagerClient() {};

  enum ControllerState
  {
    STOPPED,
    RUNNING
  };
  /*!
   * @param controller_names
   * @param controller_states
   * @return True on success, otherwise False
   */
  bool getControllerList(std::vector<std::string>& controller_names,
                         std::vector<ControllerState>& controller_states);

  /*!
   * @param controller_name
   * @return True on success, otherwise False
   */
  bool loadController(const std::string controller_name);

  /*!
   * @param controller_name
   * @return True on success, otherwise False
   */
  bool unloadController(const std::string controller_name);

  /*!
   * @param start_controller_names
   * @param stop_controller_names
   * @return True on success, otherwise False
   */
  bool switchControllers(const std::vector<std::string>& start_controller_names,
                         const std::vector<std::string>& stop_controller_names);

  /*!
   * @param start_controller_name
   * @param stop_controller_name
   * @return True on success, otherwise False
   */
  bool switchController(const std::string& start_controller_name,
                        const std::string& stop_controller_name);

  /*!
   * @param controller_name
   * @return True on success, otherwise False
   */
  bool startController(const std::string& controller_name);

  /*!
   * @param controller_name
   * @return True on success, otherwise False
   */
  bool stopController(const std::string& controller_name);

  /*!
   * @return True on success, otherwise False
   */
  bool reloadControllers();

private:

  ros::NodeHandle node_handle_;

  // services to work with controllers
  ros::ServiceClient list_controllers_service_client_;
  ros::ServiceClient switch_controller_service_client_;
  ros::ServiceClient load_controller_service_client_;
  ros::ServiceClient unload_controller_service_client_;
  ros::ServiceClient reload_libraries_controllers_service_client_;

  bool readParams();

  std::string right_arm_dmp_ik_controller_name_;
  std::string left_arm_dmp_ik_controller_name_;
  std::string dual_arm_dmp_ik_controller_name_;

};

}

#endif /* PR2_CONTROLLER_MANAGER_CLIENT_H_ */
