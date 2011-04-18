/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		icra2009_transformation_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

#ifndef ICRA2009_TRANSFORMATION_SYSTEM_BASE_H_
#define ICRA2009_TRANSFORMATION_SYSTEM_BASE_H_

// system includes
#include <boost/shared_ptr.hpp>

// local includes
#include <dmp_lib/transformation_system.h>
#include <dmp_lib/canonical_system_parameters.h>

#include <dmp_lib/icra2009_transformation_system_parameters.h>
#include <dmp_lib/icra2009_transformation_system_state.h>
#include <dmp_lib/icra2009_canonical_system_state.h>

namespace dmp_lib
{

/*!
 */
class ICRA2009TransformationSystem : public TransformationSystem
{

public:

  /*! Constructor
   */
  ICRA2009TransformationSystem() {};

  /*! Destructor
   */
  virtual ~ICRA2009TransformationSystem() {};

  /*! Assignment operator
   */
  ICRA2009TransformationSystem& operator=(const ICRA2009TransformationSystem& icra2009ts);

  /*!
   * @param parameters
   * @param states
   * @return True on success, otherwise False
   */
  bool initialize(const std::vector<ICRA2009TSParamPtr> parameters,
                  const std::vector<ICRA2009TSStatePtr> states,
                  IntegrationMethod integration_method);

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool initialize(const ICRA2009TSParamPtr parameters,
                  const ICRA2009TSStatePtr state,
                  IntegrationMethod integration_method);

  /*!
   * @param parameters
   * @param states
   * @return True on success, otherwise False
   */
  bool get(std::vector<ICRA2009TSParamConstPtr>& parameters,
           std::vector<ICRA2009TSStateConstPtr>& states) const;

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool get(ICRA2009TSParamConstPtr& parameters,
           ICRA2009TSStateConstPtr& state) const;

  /*!
   */
  void reset();

  /*! Implementes derived function
   * @param target_states
   * @param canonical_system_state
   * @return True on success, otherwise False
   */
  bool integrateAndFit(const std::vector<State>& target_states,
                       const CSStatePtr canonical_system_state,
                       const Time& dmp_time);

  /*! Implementes derived function
   * @param canonical_system_state
   * @param dmp_time
   * @param feedback
   * @param num_iterations
   * @return False if the lwr model could not come up with a prediction for various reasons, otherwise True.
   * REAL-TIME REQUIREMENTS
   */
  bool integrate(const CSStatePtr canonical_system_state,
                 const Time& dmp_time,
                 const Eigen::VectorXd& feedback,
                 const int num_iterations = 1);

  /*! Returns the version string
   * @return version string
   */
  std::string getVersionString() const
  {
    return "ICRA2009";
  }

  static double getDGain(const double k_gain)
  {
    return k_gain / 4.0;
  }

private:

  /*!
   */
  std::vector<ICRA2009TSParamPtr> parameters_;

  /*!
   */
  std::vector<ICRA2009TSStatePtr> states_;

};

/*! Abbreviation for convinience
 */
typedef ICRA2009TransformationSystem ICRA2009TS;
typedef boost::shared_ptr<ICRA2009TS> ICRA2009TSPtr;
typedef boost::shared_ptr<ICRA2009TS const> ICRA2009TSConstPtr;

}

#endif /* ICRA2009_TRANSFORMATION_SYSTEM_BASE_H_ */
