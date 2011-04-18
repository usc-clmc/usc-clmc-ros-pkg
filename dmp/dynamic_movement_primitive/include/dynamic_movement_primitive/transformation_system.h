/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		transformation_system.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 7, 2010

 *********************************************************************/

#ifndef TRANSFORMATION_SYSTEM_H_
#define TRANSFORMATION_SYSTEM_H_

// system includes
#include <ros/ros.h>
#include <string>
#include <boost/shared_ptr.hpp>

#include <dmp_lib/transformation_system.h>

// local includes
#include <dynamic_movement_primitive/TransformationSystemMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::TransformationSystemMsg TSMsg;
typedef dynamic_movement_primitive::TransformationSystemParametersMsg TSParamMsg;
typedef dynamic_movement_primitive::TransformationSystemStateMsg TSStateMsg;


class TransformationSystem
{

  /*! Allow the DynamicMovementPrimitive class to access private member variables directly
   */
  friend class DynamicMovementPrimitive;

public:

  /*!
   * @param transformation_system
   * @param ts_xml
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(dmp_lib::TSParamPtr parameters,
                                 XmlRpc::XmlRpcValue& ts_xml,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param parameters
   * @param state
   * @param ts_param_msg
   * @param ts_state_msg
   * @return True if success, otherwise False
   */
  static bool initFromMessage(dmp_lib::TSParamPtr parameters,
                              dmp_lib::TSStatePtr state,
                              const TSParamMsg& ts_param_msg,
                              const TSStateMsg& ts_state_msg);

  //  /*!
  //   * @param transformation_system
  //   * @param ts_msg
  //   * @return True if success, otherwise False
  //   */
  //  static bool initFromMessage(dmp_lib::TSPtr transformation_system,
  //                              const TSMsg& ts_msg);

  /*!
   * @param transformation_system
   * @param ts_msg
   * @return True if success, otherwise False
   */
  static bool writeToMessage(const dmp_lib::TSConstPtr transformation_system,
                             TSMsg& ts_msg);

private:

  /*! Constructor
   */
  TransformationSystem() {};

  /*! Destructor
   */
  virtual ~TransformationSystem() {};

};

/*! Abbreviation for convinience
 */
typedef TransformationSystem TS;
typedef boost::shared_ptr<TS> TSPtr;
typedef boost::shared_ptr<TS const> TSConstPtr;

}

#endif /* TRANSFORMATION_SYSTEM_H_ */
