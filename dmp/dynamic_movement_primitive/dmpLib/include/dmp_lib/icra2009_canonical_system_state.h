/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_canonical_system_state.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 7, 2010

 *********************************************************************/

#ifndef ICRA2009_CANONICAL_SYSTEM_STATE_BASE_H_
#define ICRA2009_CANONICAL_SYSTEM_STATE_BASE_H_

// system includes

// local includes
#include <dmp_lib/canonical_system_state.h>

namespace dmp_lib
{

class ICRA2009CanonicalSystemState : public CanonicalSystemState
{

  /*! Allow the CanonicalSystem class and TransformationSystem class to access private member variables directly
   */
  friend class ICRA2009TransformationSystem;
  friend class ICRA2009CanonicalSystem;

public:

  /*! Constructor
   */
  ICRA2009CanonicalSystemState() {};

  /*! Destructor
   */
  virtual ~ICRA2009CanonicalSystemState() {};

private:

};

/*! Abbreviation for convinience
 */
typedef ICRA2009CanonicalSystemState ICRA2009CSState;
typedef boost::shared_ptr<ICRA2009CSState> ICRA2009CSStatePtr;
typedef boost::shared_ptr<ICRA2009CSState const> ICRA2009CSStateConstPtr;

}

#endif /* ICRA2009_CANONICAL_SYSTEM_STATE_BASE_H_ */
