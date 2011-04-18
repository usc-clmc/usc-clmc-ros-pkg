/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		nc2010_canonical_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 9, 2010

 *********************************************************************/

#ifndef NC2010_CANONICAL_SYSTEM_H_
#define NC2010_CANONICAL_SYSTEM_H_

// system includes
#include <ros/ros.h>

#include <dmp_lib/nc2010_canonical_system.h>

// local includes
#include <dynamic_movement_primitive/canonical_system.h>
#include <dynamic_movement_primitive/NC2010CanonicalSystemMsg.h>

namespace dmp
{

/*! Abbreviation for convinience
 */
typedef dynamic_movement_primitive::NC2010CanonicalSystemMsg NC2010CSMsg;

class NC2010CanonicalSystem
{

  /*! Allow the TransformationSystem class to access private member variables directly
   */
  friend class NC2010DynamicMovementPrimitive;

public:

  typedef NC2010CanonicalSystem CS;
  typedef boost::shared_ptr<CS> CSPtr;
  typedef boost::shared_ptr<CS const> CSConstPtr;

  /*!
   * @param canonical_system
   * @param node_handle
   * @return True if success, otherwise False
   */
  static bool initFromNodeHandle(dmp_lib::NC2010CSPtr& canonical_system,
                                 ros::NodeHandle& node_handle);

  /*!
   * @param canonical_system
   * @param cs_msg
   * @return True if success, otherwise False
   */
  static bool initFromMessage(dmp_lib::NC2010CSPtr& canonical_system,
                              const NC2010CSMsg& cs_msg);

  /*!
   * @param canonical_system
   * @param cs_msg
   * @return
   */
  static bool writeToMessage(const dmp_lib::NC2010CSConstPtr canonical_system,
                             NC2010CSMsg& cs_msg);

private:

  /*! Constructor
   */
  NC2010CanonicalSystem() {};

  /*! Destructor
   */
  virtual ~NC2010CanonicalSystem() {};

};

/*! Abbreviation for convinience
 */
typedef NC2010CanonicalSystem NC2010CS;
typedef boost::shared_ptr<NC2010CS> NC2010CSPtr;
typedef boost::shared_ptr<NC2010CS const> NC2010CSConstPtr;

}

#endif /* NC2010_CANONICAL_SYSTEM_H_ */
