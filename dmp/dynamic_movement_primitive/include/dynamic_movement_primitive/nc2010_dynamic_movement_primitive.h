/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		nc2010_dynamic_movement_primitive.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 3, 2010

 *********************************************************************/

#ifndef NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_H_
#define NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_H_

// system includes
#include <vector>
#include <string>

#include <ros/ros.h>

#include <dmp_lib/nc2010_dynamic_movement_primitive.h>
#include <dmp_lib/nc2010_transformation_system.h>
#include <dmp_lib/nc2010_canonical_system.h>

// local includes
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/NC2010DynamicMovementPrimitiveMsg.h>
#include <dynamic_movement_primitive/DMPUtilitiesMsg.h>

namespace dmp
{

/*!
 */
class NC2010DynamicMovementPrimitive
{

public:

  typedef dmp_lib::NC2010DynamicMovementPrimitive DMP;
  typedef boost::shared_ptr<DMP> DMPPtr;
  typedef boost::shared_ptr<DMP const> DMPConstPtr;

  typedef dynamic_movement_primitive::NC2010DynamicMovementPrimitiveMsg DMPMsg;
  typedef boost::shared_ptr<DMPMsg> DMPMsgPtr;
  typedef boost::shared_ptr<DMPMsg const> DMPMsgConstPtr;

  /*! Initializes a DMP from parameters on the param server in the namespace of the node handle
   * @param dmp
   * @param robot_part_names
   * @param node_handle
   * @return True if initialization is successful, otherwise False
   */
  static bool initFromNodeHandle(DMPPtr& dmp,
                                 const std::vector<std::string>& robot_part_names,
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
                             DMPMsg& nc2010_dmp_msg);

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
    return dynamic_movement_primitive::DMPUtilitiesMsg::NC2010;
  }

private:

  /*! Constructor
   */
  NC2010DynamicMovementPrimitive() {};

  /*! Destructor
   */
  virtual ~NC2010DynamicMovementPrimitive() {};

};

/*! Abbreviation for convinience
 */
typedef NC2010DynamicMovementPrimitive NC2010DMP;
typedef boost::shared_ptr<NC2010DMP> NC2010DMPPtr;
typedef boost::shared_ptr<NC2010DMP const> NC2010DMPConstPtr;
typedef NC2010DynamicMovementPrimitive::DMPMsg NC2010DMPMsg;

}

#endif /* NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_H_ */
