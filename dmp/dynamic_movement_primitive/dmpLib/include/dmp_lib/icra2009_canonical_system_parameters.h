/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_canonical_system_parameters.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 6, 2010

 *********************************************************************/

#ifndef ICRA2009_CANONICAL_SYSTEM_PARAMETERS_BASE_H_
#define ICRA2009_CANONICAL_SYSTEM_PARAMETERS_BASE_H_

// system includes

// local includes
#include <dmp_lib/canonical_system_parameters.h>

namespace dmp_lib
{

/*! This class implements the parameters for a simple first order dynamical system
 */
class ICRA2009CanonicalSystemParameters : public CanonicalSystemParameters
{

    /*! Allow the CanonicalSystem class to access private member variables directly
     */
    friend class CanonicalSystem;

public:

    /*! Constructor
     */
    ICRA2009CanonicalSystemParameters() {};

    /*! Destructor
     */
    virtual ~ICRA2009CanonicalSystemParameters() {};

private:

};

/*! Abbreviation for convinience
 */
typedef ICRA2009CanonicalSystemParameters ICRA2009CSParam;
typedef boost::shared_ptr<ICRA2009CSParam> ICRA2009CSParamPtr;
typedef boost::shared_ptr<ICRA2009CSParam const> ICRA2009CSParamConstPtr;

}

#endif /* ICRA2009_CANONICAL_SYSTEM_PARAMETERS_BASE_H_ */
