/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		canonical_system_parameters.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

#ifndef CANONICAL_SYSTEM_PARAMETERS_BASE_H_
#define CANONICAL_SYSTEM_PARAMETERS_BASE_H_

// system includes
#include <boost/shared_ptr.hpp>
#include <math.h>

// local includes
#include <dmp_lib/status.h>

namespace dmp_lib
{

/*!
 */
class CanonicalSystemParameters : public Status
{

  /*! Allow the CanonicalSystem class to access private member variables directly
   */
  friend class CanonicalSystem;

public:

  /*! Constructor
   */
  CanonicalSystemParameters() :
    alpha_x_(0) {};

  /*! Destructor
   */
  virtual ~CanonicalSystemParameters() {};

  /*!
   * @param params
   * @return True if equal, otherwise False
   */
  bool operator==(const CanonicalSystemParameters &params) const
  {
    return ((isInitialized() && params.isInitialized()) && (fabs(alpha_x_ - params.alpha_x_) < EQUALITY_PRECISSION));
  }
  bool operator!=(const CanonicalSystemParameters &params) const
  {
    return !(*this == params);
  }

  /*!
   * @param alpha_x
   * @return True on success, otherwise False
   */
  bool initialize(const double alpha_x);

  /*!
   * @param other_parameters
   * @return
   */
  bool isCompatible(const CanonicalSystemParameters& other_parameters) const;

  /*!
   * @param alpha_x
   * @return True on success, otherwise False
   */
  bool get(double& alpha_x) const;

  /*! Compute alpha_x such that the canonical system drops below the cutoff when the trajectory has finished
   * @param cutoff
   * @return True on success, otherwise False
   */
  bool setCutoff(const double cutoff);

  /*!
   * @return
   */
  double getAlphaX() const;

protected:

  static const double EQUALITY_PRECISSION = 1e-6;

  /*! Time constant
   */
  double alpha_x_;

};

/*! Abbreviation for convinience
 */
typedef CanonicalSystemParameters CSParam;
typedef boost::shared_ptr<CSParam> CSParamPtr;
typedef boost::shared_ptr<CSParam const> CSParamConstPtr;

// Inline functions follow
inline double CanonicalSystemParameters::getAlphaX() const
{
  assert(initialized_);
  return alpha_x_;
}
inline bool CanonicalSystemParameters::get(double& alpha_x) const
{
  assert(initialized_);
  alpha_x = alpha_x_;
  return true;
}

}

#endif /* CANONICAL_SYSTEM_PARAMETERS_BASE_H_ */
