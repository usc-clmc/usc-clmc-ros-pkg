/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		nc2010_canonical_system_state.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 7, 2010

 *********************************************************************/

#ifndef NC2010_CANONICAL_SYSTEM_STATE_BASE_H_
#define NC2010_CANONICAL_SYSTEM_STATE_BASE_H_

// system includes

// local includes
#include <dmp_lib/canonical_system_state.h>

namespace dmp_lib
{

class NC2010CanonicalSystemState : public CanonicalSystemState
{

  /*! Allow the CanonicalSystem class and TransformationSystem class to access private member variables directly
   */
  friend class NC2010TransformationSystem;
  friend class NC2010CanonicalSystem;

public:

  /*! Constructor
   */
  NC2010CanonicalSystemState() {};

  /*! Destructor
   */
  virtual ~NC2010CanonicalSystemState() {};

private:

};

/*! Abbreviation for convinience
 */
typedef NC2010CanonicalSystemState NC2010CSState;
typedef boost::shared_ptr<NC2010CSState> NC2010CSStatePtr;
typedef boost::shared_ptr<NC2010CSState const> NC2010CSStateConstPtr;

}

#endif /* NC2010_CANONICAL_SYSTEM_STATE_BASE_H_ */
