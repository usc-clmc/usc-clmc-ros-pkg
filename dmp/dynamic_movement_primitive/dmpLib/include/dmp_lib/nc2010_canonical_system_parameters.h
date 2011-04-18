/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		nc2010_canonical_system_parameters.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 6, 2010

 *********************************************************************/

#ifndef NC2010_CANONICAL_SYSTEM_PARAMETERS_BASE_H_
#define NC2010_CANONICAL_SYSTEM_PARAMETERS_BASE_H_

// system includes

// local includes
#include <dmp_lib/canonical_system_parameters.h>

namespace dmp_lib
{

/*! This class implements the parameters for a simple first order dynamical system
 */
class NC2010CanonicalSystemParameters : public CanonicalSystemParameters
{

    /*! Allow the CanonicalSystem class to access private member variables directly
     */
    friend class CanonicalSystem;

public:

    /*! Constructor
     */
    NC2010CanonicalSystemParameters() {};

    /*! Destructor
     */
    virtual ~NC2010CanonicalSystemParameters() {};

private:

};

/*! Abbreviation for convinience
 */
typedef NC2010CanonicalSystemParameters NC2010CSParam;
typedef boost::shared_ptr<NC2010CSParam> NC2010CSParamPtr;
typedef boost::shared_ptr<NC2010CSParam const> NC2010CSParamConstPtr;

}

#endif /* NC2010_CANONICAL_SYSTEM_PARAMETERS_BASE_H_ */
