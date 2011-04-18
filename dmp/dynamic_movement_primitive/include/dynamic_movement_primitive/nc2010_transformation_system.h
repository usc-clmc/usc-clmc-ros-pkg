/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		nc2010_transformation_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 9, 2010

 *********************************************************************/

#ifndef NC2010_TRANSFORMATION_SYSTEM_H_
#define NC2010_TRANSFORMATION_SYSTEM_H_

// system includes
#include <ros/ros.h>

#include <dmp_lib/nc2010_transformation_system.h>

// local includes
#include <dynamic_movement_primitive/transformation_system.h>
#include <dynamic_movement_primitive/NC2010TransformationSystemMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::NC2010TransformationSystemMsg NC2010TSMsg;

class NC2010TransformationSystem
{
  /*! Allow the TransformationSystem class to access private member variables directly
   */
  friend class NC2010DynamicMovementPrimitive;

public:

  typedef NC2010TransformationSystem TS;
  typedef boost::shared_ptr<TS> TSPtr;
  typedef boost::shared_ptr<TS const> TSConstPtr;

  /*!
   * @param transformation_systems
   * @param robot_part_names
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(std::vector<dmp_lib::NC2010TSPtr>& transformation_systems,
                                 const std::vector<std::string>& robot_part_names,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param transformation_systems
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(std::vector<dmp_lib::NC2010TSPtr>& transformation_systems,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param transformation_systems
   * @param ts_msg
   * @return True if success, otherwise False
   */
  static bool initFromMessage(std::vector<dmp_lib::NC2010TSPtr>& transformation_systems,
                              const std::vector<NC2010TSMsg>& ts_msgs);

  /*!
   * @param transformation_systems
   * @param ts_msg
   * @return True if success, otherwise False
   */
  static bool writeToMessage(const std::vector<dmp_lib::NC2010TSConstPtr> transformation_systems,
                             std::vector<NC2010TSMsg>& ts_msgs);

private:

  /*!
   * @param transformation_systems
   * @param transformation_systems_parameters_xml
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initOneDimensionalTransformationSystemHelper(std::vector<dmp_lib::NC2010TSPtr>& transformation_systems,
                                                           XmlRpc::XmlRpcValue transformation_systems_parameters_xml,
                                                           ros::NodeHandle& node_handle,
                                                           dmp_lib::TransformationSystem::IntegrationMethod integration_method = dmp_lib::TransformationSystem::NORMAL);

  /*!
   * @param transformation_systems
   * @param transformation_systems_parameters_xml
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initMultiDimensionalTransformationSystemHelper(std::vector<dmp_lib::NC2010TSPtr>& transformation_systems,
                                                             XmlRpc::XmlRpcValue transformation_systems_parameters_xml,
                                                             ros::NodeHandle& node_handle,
                                                             dmp_lib::TransformationSystem::IntegrationMethod integration_method = dmp_lib::TransformationSystem::NORMAL);

  /*! Constructor
   */
  NC2010TransformationSystem() {};

  /*! Destructor
   */
  virtual ~NC2010TransformationSystem() {};

};

/*! Abbreviation for convinience
 */
typedef NC2010TransformationSystem NC2010TS;
typedef boost::shared_ptr<NC2010TS> NC2010TSPtr;
typedef boost::shared_ptr<NC2010TS const> NC2010TSConstPtr;

}

#endif /* NC2010_TRANSFORMATION_SYSTEM_H_ */
