/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_canonical_system.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 6, 2010

 *********************************************************************/

#ifndef ICRA2009_CANONICAL_SYSTEM_BASE_H_
#define ICRA2009_CANONICAL_SYSTEM_BASE_H_

// system include
#include <string>
#include <boost/shared_ptr.hpp>

// local include
#include <dmp_lib/canonical_system.h>
#include <dmp_lib/icra2009_canonical_system_parameters.h>
#include <dmp_lib/icra2009_canonical_system_state.h>

namespace dmp_lib
{

/*!
 */
class ICRA2009CanonicalSystem : public CanonicalSystem
{

public:

  friend class ICRA2009TransformationSystem;

  /*! Constructor
   */
  ICRA2009CanonicalSystem() {};

  /*! Destructor
   */
  virtual ~ICRA2009CanonicalSystem() {};

  /*! Assignment operator
   */
  ICRA2009CanonicalSystem& operator=(const ICRA2009CanonicalSystem& icra2009cs);

  /*! Initializes the canonical system from state and parameters
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool initialize(const ICRA2009CSParamPtr parameters,
                  const ICRA2009CSStatePtr state);

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool get(ICRA2009CSParamConstPtr& parameters,
           ICRA2009CSStateConstPtr& state) const;

  /*! Reset the canonical system
   */
  void reset();

  /*! Integrate the canonical system
   * @param dmp_time
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool integrate(const Time& dmp_time);

  /*! Returns the time (in sec) of the movement.
   * Will stop when the movement duration is reached
   * @return
   * REAL-TIME REQUIREMENTS
   */
  double getTime() const;

  /*!
   * @param num_time_steps
   * @param rollout
   * @param cutoff
   * @return True on success, otherwise False
   */
  bool getRollout(const int num_time_steps,
                  const double cutoff,
                  Eigen::VectorXd& rollout) const;

  /*! Returns the version string
   * @return version string
   */
  std::string getVersionString() const
  {
    return "ICRA2009";
  }

private:

  /*!
   */
  ICRA2009CSParamPtr parameters_;

  /*!
   */
  ICRA2009CSStatePtr state_;

};

/*! Abbreviation for convinience
 */
typedef ICRA2009CanonicalSystem ICRA2009CS;
typedef boost::shared_ptr<ICRA2009CS> ICRA2009CSPtr;
typedef boost::shared_ptr<ICRA2009CS const> ICRA2009CSConstPtr;

}

#endif /* ICRA2009_CANONICAL_SYSTEM_BASE_H_ */
