/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		icra2009_transformation_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 9, 2010

 *********************************************************************/

#ifndef ICRA2009_TRANSFORMATION_SYSTEM_H_
#define ICRA2009_TRANSFORMATION_SYSTEM_H_

// system includes
#include <ros/ros.h>

#include <dmp_lib/icra2009_transformation_system.h>

// local includes
#include <dynamic_movement_primitive/transformation_system.h>
#include <dynamic_movement_primitive/ICRA2009TransformationSystemMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::ICRA2009TransformationSystemMsg ICRA2009TSMsg;

class ICRA2009TransformationSystem
{
  /*! Allow the TransformationSystem class to access private member variables directly
   */
  friend class ICRA2009DynamicMovementPrimitive;

public:

  typedef ICRA2009TransformationSystem TS;
  typedef boost::shared_ptr<TS> TSPtr;
  typedef boost::shared_ptr<TS const> TSConstPtr;

  /*!
   * @param transformation_systems
   * @param robot_part_names
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                 const std::vector<std::string>& robot_part_names,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param transformation_systems
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param transformation_systems
   * @param ts_msg
   * @return True if success, otherwise False
   */
  static bool initFromMessage(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                              const std::vector<ICRA2009TSMsg>& ts_msgs);

  /*!
   * @param transformation_systems
   * @param ts_msg
   * @return True if success, otherwise False
   */
  static bool writeToMessage(const std::vector<dmp_lib::ICRA2009TSConstPtr> transformation_systems,
                             std::vector<ICRA2009TSMsg>& ts_msgs);

private:

  /*!
   * @param transformation_systems
   * @param transformation_systems_parameters_xml
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initOneDimensionalTransformationSystemHelper(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                                           XmlRpc::XmlRpcValue transformation_systems_parameters_xml,
                                                           ros::NodeHandle& node_handle,
                                                           dmp_lib::TransformationSystem::IntegrationMethod integration_method = dmp_lib::TransformationSystem::NORMAL);

  /*!
   * @param transformation_systems
   * @param transformation_systems_parameters_xml
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initMultiDimensionalTransformationSystemHelper(std::vector<dmp_lib::ICRA2009TSPtr>& transformation_systems,
                                                             XmlRpc::XmlRpcValue transformation_systems_parameters_xml,
                                                             ros::NodeHandle& node_handle,
                                                             dmp_lib::TransformationSystem::IntegrationMethod integration_method = dmp_lib::TransformationSystem::NORMAL);

  /*! Constructor
   */
  ICRA2009TransformationSystem() {};

  /*! Destructor
   */
  virtual ~ICRA2009TransformationSystem() {};

};

/*! Abbreviation for convinience
 */
typedef ICRA2009TransformationSystem ICRA2009TS;
typedef boost::shared_ptr<ICRA2009TS> ICRA2009TSPtr;
typedef boost::shared_ptr<ICRA2009TS const> ICRA2009TSConstPtr;

}

#endif /* ICRA2009_TRANSFORMATION_SYSTEM_H_ */
