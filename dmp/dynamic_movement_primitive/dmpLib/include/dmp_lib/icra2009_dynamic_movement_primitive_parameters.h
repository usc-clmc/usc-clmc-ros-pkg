/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		icra2009_dynamic_movement_primitive_parameters.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 9, 2010

 *********************************************************************/

#ifndef ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H_
#define ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H_

// system includes

// local includes
#include <dmp_lib/dynamic_movement_primitive_parameters.h>

namespace dmp_lib
{

class ICRA2009DynamicMovementPrimitiveParameters : public DynamicMovementPrimitiveParameters
{

public:

  /*! Constructor
   */
  ICRA2009DynamicMovementPrimitiveParameters() {};

  /*! Destructor
   */
  virtual ~ICRA2009DynamicMovementPrimitiveParameters() {};

private:

};

/*! Abbreviation for convinience
 */
typedef ICRA2009DynamicMovementPrimitiveParameters ICRA2009DMPParam;
typedef boost::shared_ptr<ICRA2009DMPParam> ICRA2009DMPParamPtr;
typedef boost::shared_ptr<ICRA2009DMPParam const> ICRA2009DMPParamConstPtr;

}

#endif /* ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H_ */
