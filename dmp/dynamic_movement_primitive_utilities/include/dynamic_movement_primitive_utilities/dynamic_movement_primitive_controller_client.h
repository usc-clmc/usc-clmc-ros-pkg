/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_controller_client.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 24, 2011

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_CLIENT_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_CLIENT_H_

// system includes
#include <ros/ros.h>

#include <usc_utilities/assert.h>

#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/ControllerStatusMsg.h>

#include <visualization_utilities/robot_pose_visualizer.h>

// local includes
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_base_client.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h>

namespace dmp_utilities
{

class DynamicMovementPrimitiveControllerClient
{

public:

  /*! Constructor
   */
  DynamicMovementPrimitiveControllerClient() :
    initialized_(false), current_controller_("NoController") {};//, goal_configuration_visualizer_("/dmp_goal_configuration") {};

  /*! Destructor
   */
  virtual ~DynamicMovementPrimitiveControllerClient() {};

  /*!
   * @param controller_name
   * @return True on success, otherwise False
   */
  bool initialize(// const std::string& robot_part_name,
                  // const std::string& base_frame_id,
                  const std::string& controller_name);

  /*!
   * @param robot_part_name
   * @param base_frame_id
   * @param controller_names
   * @param controller_namespace
   * @return True on success, otherwise False
   */
  bool initialize(const std::string& robot_part_name,
                  // const std::string& base_frame_id,
                  const std::vector<std::string>& controller_names,
                  const std::string& controller_namespace);

  /**
   * Sets this client to single-threaded mode. This means that if the waitForCompletion()
   * function is called, it will spin when in single-threaded mode to listen for status messages,
   * otherwise callbacks are expected to be processed separately.
   */
  void setSingleThreadedMode(bool single_threaded_mode = true);

  /*!
   * @param msg
   * @param wait_for_success
   * @return True on success, otherwise False
   */
  bool sendCommand(const dmp::ICRA2009DMPMsg& msg,
                   const std::string& controller_name,
                   bool wait_for_success = true);

  /*!
   * @param msg
   * @param wait_for_success
   * @return True on success, otherwise False
   */
  bool sendCommand(const dmp::NC2010DMPMsg& msg,
                   const std::string& controller_name,
                   bool wait_for_success = true);

  /**
   * @param dmp
   * @param wait_for_success
   * @return True if command was published successfully, False if not
   */
  bool sendCommand(const dmp_lib::DMPPtr& dmp,
                   const std::string& controller_name,
                   bool wait_for_success = true);

  /**
   * @return True if controller is active, False if not
   */
  bool isActive();

  /**
   * Gets the status of the last executed DMP
   *
   * @return False if no DMP has ever been executed through this client
   */
  bool getControllerStatus(dynamic_movement_primitive::ControllerStatusMsg& dmp_status);

  /**
   * Wait for completion of the last sent command.
   *
   * @return True on success, False if the command was preempted or timed out
   */
  bool waitForCompletion();

  /**
   * Halts the currently executing trajectory immediately
   * @return
   */
  bool halt(bool wait_for_success = true);

private:

  /*!
   */
  bool initialized_;

  /*!
   */
  typedef DynamicMovementPrimitiveControllerBaseClient<dmp::ICRA2009DMP, dmp::ICRA2009DMPMsg> DMPControllerClient;
  typedef boost::shared_ptr<DMPControllerClient> DMPControllerClientPtr;
  typedef std::pair<std::string, DMPControllerClientPtr> ControllerMapPair;
  typedef std::map<std::string, DMPControllerClientPtr>::iterator ControllerMapIterator;

  /*!
   */
  std::map<std::string, DMPControllerClientPtr> icra2009_controller_clients_;

  /*!
   */
  std::vector<std::string> controller_names_;
  std::string current_controller_;

  /*!
   */
  // std::string robot_part_name_;
  // std::string base_frame_id_;

  /*!
   * @param controller
   * @return
   */
  bool switchController(const std::string& controller_name);

  enum Configuration
  {
    START,
    GOAL
  };

  /*!
   * @param configuration
   * @return
   */
  bool publishConfiguration(const Configuration configuration);

  /*!
   */
  dynamic_movement_primitive::ControllerStatusMsg last_dmp_status_;

  /*!
   */
  // visualization_utilities::RobotPoseVisualizer goal_configuration_visualizer_;
  void setRobotVisualizer();

};

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_CLIENT_H_ */
