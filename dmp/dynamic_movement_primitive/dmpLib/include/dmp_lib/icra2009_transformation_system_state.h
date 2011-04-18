/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		icra2009_transformation_system_state.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 7, 2010

 *********************************************************************/

#ifndef ICRA2009_TRANSFORMATION_SYSTEM_STATE_BASE_H_
#define ICRA2009_TRANSFORMATION_SYSTEM_STATE_BASE_H_

// system includes

// local includes
#include <dmp_lib/transformation_system_state.h>

namespace dmp_lib
{

class ICRA2009TransformationSystemState : public TransformationSystemState
{

  /*! Allow the ICRA2009TransformationSystem class to access private member variables directly
   */
  friend class ICRA2009TransformationSystem;

public:

  /*! Constructor
   */
  ICRA2009TransformationSystemState() {};

  /*! Destructor
   */
  virtual ~ICRA2009TransformationSystemState() {};

private:

};

/*! Abbreviation for convinience
 */
typedef ICRA2009TransformationSystemState ICRA2009TSState;
typedef boost::shared_ptr<ICRA2009TSState> ICRA2009TSStatePtr;
typedef boost::shared_ptr<ICRA2009TSState const> ICRA2009TSStateConstPtr;

}

#endif /* ICRA2009_TRANSFORMATION_SYSTEM_STATE_BASE_H_ */
