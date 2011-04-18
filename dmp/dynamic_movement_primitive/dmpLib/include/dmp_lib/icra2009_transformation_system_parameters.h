/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		icra2009_transformation_system_parameters.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

#ifndef ICRA2009_TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_
#define ICRA2009_TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_

// system includes
#include <lwr_lib/lwr.h>

// local includes
#include <dmp_lib/transformation_system_parameters.h>

namespace dmp_lib
{

/*!
 */
class ICRA2009TransformationSystemParameters : public TransformationSystemParameters
{

  /*! Allow the ICRA2009TransformationSystem class to access private member variables directly
   */
  friend class ICRA2009TransformationSystem;
  friend class ICRA2009DynamicMovementPrimitive;

public:

  /*! Constructor
   */
  ICRA2009TransformationSystemParameters() {};

  /*! Destructor
   */
  virtual ~ICRA2009TransformationSystemParameters() {};

  /*!
   * @param k_gain
   * @param d_gain
   * @return True on success, otherwise False
   */
  bool initialize(const double k_gain,
                  const double d_gain);

  /*! Initializes the entire transformation system (including the base class)
   * @param lwr_model
   * @param name
   * @param k_gain
   * @param d_gain
   * @return True on success, otherwise False
   */
  bool initialize(const lwr_lib::LWRPtr lwr_model,
                  const std::string& name,
                  const double k_gain,
                  const double d_gain,
                  const double initial_start = 0,
                  const double initial_goal = 0);

  /*!
   * @param k_gain
   * @param d_gain
   * @return True on success, otherwise False
   */
  bool get(double& k_gain,
           double& d_gain) const;

private:

  /*!
   */
  double k_gain_;

  /*!
   */
  double d_gain_;

};

/*! Abbreviation for convinience
 */
typedef ICRA2009TransformationSystemParameters ICRA2009TSParam;
typedef boost::shared_ptr<ICRA2009TSParam> ICRA2009TSParamPtr;
typedef boost::shared_ptr<ICRA2009TSParam const> ICRA2009TSParamConstPtr;

}

#endif /* ICRA2009_TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_ */
