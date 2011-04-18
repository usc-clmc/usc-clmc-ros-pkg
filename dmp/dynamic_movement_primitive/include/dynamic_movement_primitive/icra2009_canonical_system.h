/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		icra2009_canonical_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 9, 2010

 *********************************************************************/

#ifndef ICRA2009_CANONICAL_SYSTEM_H_
#define ICRA2009_CANONICAL_SYSTEM_H_

// system includes
#include <ros/ros.h>

#include <dmp_lib/icra2009_canonical_system.h>

// local includes
#include <dynamic_movement_primitive/canonical_system.h>
#include <dynamic_movement_primitive/ICRA2009CanonicalSystemMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::ICRA2009CanonicalSystemMsg ICRA2009CSMsg;

class ICRA2009CanonicalSystem
{

  /*! Allow the TransformationSystem class to access private member variables directly
   */
  friend class ICRA2009DynamicMovementPrimitive;

public:

  typedef ICRA2009CanonicalSystem CS;
  typedef boost::shared_ptr<CS> CSPtr;
  typedef boost::shared_ptr<CS const> CSConstPtr;

  /*!
   * @param canonical_system
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(dmp_lib::ICRA2009CSPtr& canonical_system,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param canonical_system
   * @param cs_msg
   * @return True if success, otherwise False
   */
  static bool initFromMessage(dmp_lib::ICRA2009CSPtr& canonical_system,
                              const ICRA2009CSMsg& cs_msg);

  /*!
   * @param canonical_system
   * @param cs_msg
   * @return
   */
  static bool writeToMessage(const dmp_lib::ICRA2009CSConstPtr canonical_system,
                             ICRA2009CSMsg& cs_msg);

private:

  /*! Constructor
   */
  ICRA2009CanonicalSystem() {};

  /*! Destructor
   */
  virtual ~ICRA2009CanonicalSystem() {};

};

/*! Abbreviation for convinience
 */
typedef ICRA2009CanonicalSystem ICRA2009CS;
typedef boost::shared_ptr<ICRA2009CS> ICRA2009CSPtr;
typedef boost::shared_ptr<ICRA2009CS const> ICRA2009CSConstPtr;

}

#endif /* ICRA2009_CANONICAL_SYSTEM_H_ */
