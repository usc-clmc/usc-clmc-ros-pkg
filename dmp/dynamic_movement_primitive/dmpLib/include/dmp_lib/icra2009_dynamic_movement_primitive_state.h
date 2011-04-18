/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_dynamic_movement_primitive_state.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 9, 2010

 *********************************************************************/

#ifndef ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_BASE_H_
#define ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_BASE_H_

// system includes

// local includes
#include <dmp_lib/dynamic_movement_primitive_state.h>

namespace dmp_lib
{

class ICRA2009DynamicMovementPrimitiveState : public DynamicMovementPrimitiveState
{

public:

    /*! Constructor
     */
    ICRA2009DynamicMovementPrimitiveState() {};

    /*! Destructor
     */
    virtual ~ICRA2009DynamicMovementPrimitiveState() {};

private:

};

/*! Abbreviation for convinience
 */
typedef ICRA2009DynamicMovementPrimitiveState ICRA2009DMPState;
typedef boost::shared_ptr<ICRA2009DMPState> ICRA2009DMPStatePtr;
typedef boost::shared_ptr<ICRA2009DMPState const> ICRA2009DMPStateConstPtr;

}

#endif /* ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_H_ */
