/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		icra2009_dynamic_movement_primitive.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 3, 2010

 *********************************************************************/

#ifndef ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_H_
#define ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_H_

// system includes
#include <vector>
#include <string>

#include <ros/ros.h>

#include <dmp_lib/icra2009_dynamic_movement_primitive.h>
#include <dmp_lib/icra2009_transformation_system.h>
#include <dmp_lib/icra2009_canonical_system.h>

// local includes
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/ICRA2009DynamicMovementPrimitiveMsg.h>
#include <dynamic_movement_primitive/DMPUtilitiesMsg.h>

namespace dmp
{

/*!
 */
class ICRA2009DynamicMovementPrimitive
{

public:

  typedef dmp_lib::ICRA2009DynamicMovementPrimitive DMP;
  typedef boost::shared_ptr<DMP> DMPPtr;
  typedef boost::shared_ptr<DMP const> DMPConstPtr;

  typedef dynamic_movement_primitive::ICRA2009DynamicMovementPrimitiveMsg DMPMsg;
  typedef boost::shared_ptr<DMPMsg> DMPMsgPtr;
  typedef boost::shared_ptr<DMPMsg const> DMPMsgConstPtr;

  /*! Initializes a DMP from parameters on the param server in the namespace of the node handle
   * The transformation system will be initialized using the provided robot part names
   * @param dmp
   * @param robot_part_names
   * @param node_handle
   * @return True if initialization is successful, otherwise False
   */
  static bool initFromNodeHandle(DMPPtr& dmp,
                                 const std::vector<std::string>& robot_part_names,
                                 ros::NodeHandle& node_handle);

  /*! Initializes a DMP from parameters on the param server in the namespace of the node handle
   * @param dmp
   * @param node_handle
   * @return True if initialization is successful, otherwise False
   */
  static bool initFromNodeHandle(DMPPtr& dmp,
                                 ros::NodeHandle& node_handle);

  /*! Initializes a DMP from a message
   * @param dmp (output)
   * @param dmp_msg (input)
   * @return True if initialization is successful, otherwise False
   */
  static bool createFromMessage(DMPPtr& dmp,
                                const DMPMsg& dmp_msg);

  /*! Initializes a DMP from a message
   * @param dmp (output)
   * @param dmp_msg (input)
   * @return True if initialization is successful, otherwise False
   */
  static bool initFromMessage(const DMPPtr dmp,
                              const DMPMsg& dmp_msg);

  /*!
   * @param dmp_msg
   * @return True if successful, otherwise False
   */
  static bool writeToMessage(const DMPConstPtr dmp,
                             DMPMsg& icra2009_dmp_msg);

  /*!
   * @param dmp
   * @param abs_bag_file_name
   * @return True if successful, otherwise False
   */
  static bool readFromDisc(DMPPtr& dmp,
                           const std::string& abs_bag_file_name);

  /*!
   * @param dmp
   * @param abs_bag_file_name
   * @return
   */
  static bool writeToDisc(const DMPConstPtr dmp,
                          const std::string& abs_bag_file_name);

  /*!
   * @return
   */
  static std::string getVersionString()
  {
    return dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009;
  }

private:

  /*! Constructor
   */
  ICRA2009DynamicMovementPrimitive() {};

  /*! Destructor
   */
  virtual ~ICRA2009DynamicMovementPrimitive() {};

};

/*! Abbreviation for convinience
 */
typedef ICRA2009DynamicMovementPrimitive ICRA2009DMP;
typedef boost::shared_ptr<ICRA2009DMP> ICRA2009DMPPtr;
typedef boost::shared_ptr<ICRA2009DMP const> ICRA2009DMPConstPtr;
typedef ICRA2009DynamicMovementPrimitive::DMPMsg ICRA2009DMPMsg;

}

#endif /* ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_H_ */
