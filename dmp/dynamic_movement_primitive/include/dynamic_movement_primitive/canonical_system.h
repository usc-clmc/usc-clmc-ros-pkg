/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		canonical_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 7, 2010

 *********************************************************************/

#ifndef CANONICAL_SYSTEM_H_
#define CANONICAL_SYSTEM_H_

// system includes
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <dmp_lib/canonical_system.h>

// local includes
#include <dynamic_movement_primitive/CanonicalSystemMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::CanonicalSystemMsg CSMsg;

class CanonicalSystem
{

  /*! Allow the DynamicMovementPrimitive class to access private member variables directly
   */
  friend class DynamicMovementPrimitive;

public:

  /*!
   * @param canonical_system
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(dmp_lib::CSPtr canonical_system,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param canonical_system
   * @param cs_msg
   * @return True if success, otherwise False
   */
  static bool initFromMessage(dmp_lib::CSPtr canonical_system,
                              const CSMsg& cs_msg);

  /*!
   *
   * @param canonical_system
   * @param cs_msg
   * @return True if success, otherwise False
   */
  static bool writeToMessage(const dmp_lib::CSConstPtr canonical_system,
                             CSMsg& cs_msg);

private:

  /*! Constructor
   */
  CanonicalSystem() {};

  /*! Destructor
   */
  virtual ~CanonicalSystem() {};

};

/*! Abbreviation for convenience
 */
typedef CanonicalSystem CS;
typedef boost::shared_ptr<CS> CSPtr;
typedef boost::shared_ptr<CS const> CSConstPtr;

}

#endif /* CANONICAL_SYSTEM_H_ */
