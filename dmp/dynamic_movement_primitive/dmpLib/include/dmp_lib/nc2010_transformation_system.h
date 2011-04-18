/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		nc2010_transformation_system.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

#ifndef NC2010_TRANSFORMATION_SYSTEM_BASE_H_
#define NC2010_TRANSFORMATION_SYSTEM_BASE_H_

// system includes
#include <boost/shared_ptr.hpp>

// local includes
#include <dmp_lib/transformation_system.h>
#include <dmp_lib/canonical_system_parameters.h>

#include <dmp_lib/nc2010_transformation_system_parameters.h>
#include <dmp_lib/nc2010_transformation_system_state.h>
#include <dmp_lib/nc2010_canonical_system_state.h>

namespace dmp_lib
{

/*!
 */
class NC2010TransformationSystem : public TransformationSystem
{

public:

  /*! Constructor
   */
  NC2010TransformationSystem() {};

  /*! Destructor
   */
  virtual ~NC2010TransformationSystem() {};

  /*! Assignment operator
   */
  NC2010TransformationSystem& operator=(const NC2010TransformationSystem& nc2010ts);

  /*!
   * @param parameters
   * @param states
   * @return True on success, otherwise False
   */
  bool initialize(const std::vector<NC2010TSParamPtr> parameters,
                  const std::vector<NC2010TSStatePtr> states,
                  IntegrationMethod integration_method);

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool initialize(const NC2010TSParamPtr parameters,
                  const NC2010TSStatePtr state,
                  IntegrationMethod integration_method);

  /*!
   * @param parameters
   * @param states
   * @return True on success, otherwise False
   */
  bool get(std::vector<NC2010TSParamConstPtr>& parameters,
           std::vector<NC2010TSStateConstPtr>& states) const;

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool get(NC2010TSParamConstPtr& parameters,
           NC2010TSStateConstPtr& state) const;

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
    return "NC2010";
  }

  static double getDGain(const double k_gain)
  {
    return k_gain / 4.0;
  }

private:

  /*!
   */
  std::vector<NC2010TSParamPtr> parameters_;

  /*!
   */
  std::vector<NC2010TSStatePtr> states_;

};

/*! Abbreviation for convinience
 */
typedef NC2010TransformationSystem NC2010TS;
typedef boost::shared_ptr<NC2010TS> NC2010TSPtr;
typedef boost::shared_ptr<NC2010TS const> NC2010TSConstPtr;

}

#endif /* NC2010_TRANSFORMATION_SYSTEM_BASE_H_ */
