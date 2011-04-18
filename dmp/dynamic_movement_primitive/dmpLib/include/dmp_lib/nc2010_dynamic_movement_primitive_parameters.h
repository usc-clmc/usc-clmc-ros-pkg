/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		nc2010_dynamic_movement_primitive_parameters.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 9, 2010

 *********************************************************************/

#ifndef NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H_
#define NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H_

// system includes

// local includes
#include <dmp_lib/dynamic_movement_primitive_parameters.h>

namespace dmp_lib
{

class NC2010DynamicMovementPrimitiveParameters : public DynamicMovementPrimitiveParameters
{

public:

  /*! Constructor
   */
  NC2010DynamicMovementPrimitiveParameters() {};

  /*! Destructor
   */
  virtual ~NC2010DynamicMovementPrimitiveParameters() {};

private:

};

/*! Abbreviation for convinience
 */
typedef NC2010DynamicMovementPrimitiveParameters NC2010DMPParam;
typedef boost::shared_ptr<NC2010DMPParam> NC2010DMPParamPtr;
typedef boost::shared_ptr<NC2010DMPParam const> NC2010DMPParamConstPtr;

}

#endif /* NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_PARAMETERS_BASE_H_ */
