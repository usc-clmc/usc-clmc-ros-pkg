/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		nc2010_dynamic_movement_primitive_state.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 9, 2010

 *********************************************************************/

#ifndef NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_BASE_H_
#define NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_BASE_H_

// system includes

// local includes
#include <dmp_lib/dynamic_movement_primitive_state.h>

namespace dmp_lib
{

class NC2010DynamicMovementPrimitiveState : public DynamicMovementPrimitiveState
{

public:

    /*! Constructor
     */
    NC2010DynamicMovementPrimitiveState() {};

    /*! Destructor
     */
    virtual ~NC2010DynamicMovementPrimitiveState() {};

private:

};

/*! Abbreviation for convinience
 */
typedef NC2010DynamicMovementPrimitiveState NC2010DMPState;
typedef boost::shared_ptr<NC2010DMPState> NC2010DMPStatePtr;
typedef boost::shared_ptr<NC2010DMPState const> NC2010DMPStateConstPtr;

}

#endif /* NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_STATE_H_ */
