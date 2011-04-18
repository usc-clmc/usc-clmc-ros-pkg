/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		dynamic_movement_primitive.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 6, 2010

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_H_

// system includes
#include <ros/ros.h>

#include <vector>
#include <string>

#include <dmp_lib/dynamic_movement_primitive.h>

// local includes
#include <dynamic_movement_primitive/DynamicMovementPrimitiveMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::DynamicMovementPrimitiveMsg DMPMsg;

class DynamicMovementPrimitive
{

public:

    /*! Initializes a DMP from parameters on the param server in the namespace of the node handle
     * @param dmp (input)
     * @param node_handle (input)
     * @return True if initialization is successful, otherwise False
     */
    static bool initFromNodeHandle(dmp_lib::DMPPtr dmp,
                                   ros::NodeHandle& node_handle);

    /*! Initializes a DMP from a message
     * @param dmp (input)
     * @param dmp_msg (input)
     * @return True if initialization is successful, otherwise False
     */
    static bool initFromMessage(dmp_lib::DMPPtr dmp,
                                const DMPMsg& dmp_msg);

    /*!
     * @param dmp
     * @param dmp_msg
     * @return True if successful, otherwise False
     */
    static bool writeToMessage(dmp_lib::DMPConstPtr dmp,
                               DMPMsg& dmp_msg);

private:

    /*! Constructor
     */
    DynamicMovementPrimitive() {};

    /*! Destructor
     */
    virtual ~DynamicMovementPrimitive() {};

};

/*! Abbreviation for convinience
 */
typedef DynamicMovementPrimitive DMP;
typedef boost::shared_ptr<DMP> DMPPtr;
typedef boost::shared_ptr<DMP const> DMPConstPtr;

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_H_ */
