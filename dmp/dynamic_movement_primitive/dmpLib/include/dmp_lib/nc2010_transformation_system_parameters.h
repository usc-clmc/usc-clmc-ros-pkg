/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		nc2010_transformation_system_parameters.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

#ifndef NC2010_TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_
#define NC2010_TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_

// system includes
#include <lwr_lib/lwr.h>

// local includes
#include <dmp_lib/transformation_system_parameters.h>

namespace dmp_lib
{

/*!
 */
class NC2010TransformationSystemParameters : public TransformationSystemParameters
{

  /*! Allow the NC2010TransformationSystem class to access private member variables directly
   */
  friend class NC2010TransformationSystem;
  friend class NC2010DynamicMovementPrimitive;

public:

  /*! Constructor
   */
  NC2010TransformationSystemParameters() {};

  /*! Destructor
   */
  virtual ~NC2010TransformationSystemParameters() {};

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
typedef NC2010TransformationSystemParameters NC2010TSParam;
typedef boost::shared_ptr<NC2010TSParam> NC2010TSParamPtr;
typedef boost::shared_ptr<NC2010TSParam const> NC2010TSParamConstPtr;

}

#endif /* NC2010_TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_ */
