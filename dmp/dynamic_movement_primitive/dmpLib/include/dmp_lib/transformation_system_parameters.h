/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		transformation_system_parameters.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

#ifndef TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_
#define TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_

// system include
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <lwr_lib/lwr.h>

// local include
#include <dmp_lib/status.h>

namespace dmp_lib
{

/*! This class contains all variables that need to be saved to file in order to store a dmp to file
 */
class TransformationSystemParameters : public Status
{

  /*! Allow the TransformationSystem class to access private member variables directly
   */
  friend class TransformationSystem;
  friend class DynamicMovementPrimitive;

public:

  /*! Constructor
   */
  TransformationSystemParameters() :
    initial_start_(0.0), initial_goal_(0.0), name_("unnamed") {};

  /*! Destructor
   */
  virtual ~TransformationSystemParameters() {};

  /*! Assignment operator
   */
  TransformationSystemParameters& operator=(const TransformationSystemParameters& parameters);

  /*!
   * @param lwr_model
   * @param name
   * @param initial_start
   * @param initial_goal
   * @return True if success, otherwise False
   */
  bool initialize(const lwr_lib::LWRPtr lwr_model,
                  const std::string& name,
                  const double initial_start = 0,
                  const double initial_goal = 0);

  /*!
   * @param lwr_model
   * @param name
   * @param initial_start
   * @param initial_goal
   * @return True if success, otherwise False
   */
  bool get(lwr_lib::LWRConstPtr& lwr_model,
           std::string& name,
           double& initial_start,
           double& initial_goal) const;

  /*!
   * @return
   */
  double getInitialStart() const;

  /*!
   * @param initial_start
   */
  void setInitialStart(const double initial_start);

  /*!
   * @return
   */
  double getInitialGoal() const;

  /*!
   * @param initial_goal
   */
  void setInitialGoal(const double initial_goal);

  /*!
   * @return
   */
  std::string getName() const;

  /*!
   * @param name
   */
  void setName(const std::string& name);

  /*!
   * @return
   */
  const lwr_lib::LWRPtr getLWRModel() const
  {
    return lwr_model_;
  }

protected:

  /*!
   */
  lwr_lib::LWRPtr lwr_model_;

  /*!
   */
  double initial_start_;
  double initial_goal_;

  /*!
   */
  std::string name_;

private:

};

/*! Abbreviation for convinience
 */
typedef TransformationSystemParameters TSParam;
typedef boost::shared_ptr<TSParam> TSParamPtr;
typedef boost::shared_ptr<TSParam const> TSParamConstPtr;

}

#endif /* TRANSFORMATION_SYSTEM_PARAMETERS_BASE_H_ */
