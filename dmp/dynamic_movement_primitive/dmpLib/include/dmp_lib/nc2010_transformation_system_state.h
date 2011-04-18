/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		nc2010_transformation_system_state.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 7, 2010

 *********************************************************************/

#ifndef NC2010_TRANSFORMATION_SYSTEM_STATE_BASE_H_
#define NC2010_TRANSFORMATION_SYSTEM_STATE_BASE_H_

// system includes

// local includes
#include <dmp_lib/transformation_system_state.h>

namespace dmp_lib
{

class NC2010TransformationSystemState : public TransformationSystemState
{

  /*! Allow the NC2010TransformationSystem class to access private member variables directly
   */
  friend class NC2010TransformationSystem;

public:

  /*! Constructor
   */
  NC2010TransformationSystemState() {};

  /*! Destructor
   */
  virtual ~NC2010TransformationSystemState() {};

private:

};

/*! Abbreviation for convinience
 */
typedef NC2010TransformationSystemState NC2010TSState;
typedef boost::shared_ptr<NC2010TSState> NC2010TSStatePtr;
typedef boost::shared_ptr<NC2010TSState const> NC2010TSStateConstPtr;

}

#endif /* NC2010_TRANSFORMATION_SYSTEM_STATE_BASE_H_ */
