/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_dynamic_movement_primitive.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 22, 2010

 *********************************************************************/

#ifndef ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_
#define ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_

// system includes
#include <vector>
#include <string>

// #include <lwr_lib/lwr_parameters.h>

// local includes
#include <dmp_lib/dynamic_movement_primitive.h>
#include <dmp_lib/icra2009_dynamic_movement_primitive_state.h>
#include <dmp_lib/icra2009_dynamic_movement_primitive_parameters.h>
#include <dmp_lib/icra2009_transformation_system.h>
#include <dmp_lib/icra2009_canonical_system.h>

namespace dmp_lib
{

class ICRA2009DynamicMovementPrimitive : public DynamicMovementPrimitive
{

public:

  /*! Constructor
   */
  ICRA2009DynamicMovementPrimitive() {};

  /*! Destructor
   */
  virtual ~ICRA2009DynamicMovementPrimitive() {};

  /*! Assignment operator
   * @param icra2009dmp
   * @return
   */
  ICRA2009DynamicMovementPrimitive& operator=(const ICRA2009DynamicMovementPrimitive& icra2009dmp);

  /*!
   * @param parameters
   * @param state
   * @param transformation_systems
   * @param canonical_system
   * @return True on success, otherwise False
   */
  bool initialize(ICRA2009DMPParamPtr& parameters,
                  ICRA2009DMPStatePtr& state,
                  std::vector<ICRA2009TSPtr>& transformation_systems,
                  ICRA2009CSPtr& canonical_system);

  /*!
   * @param icra2009_dmp
   * @param check_for_compatibiliy
   * @return
   */
  bool add(const ICRA2009DynamicMovementPrimitive& icra2009_dmp,
           bool check_for_compatibiliy = true);

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool get(ICRA2009DMPParamConstPtr& parameters,
           ICRA2009DMPStateConstPtr& state,
           std::vector<ICRA2009TSConstPtr>& transformation_systems,
           ICRA2009CSConstPtr& canonical_system) const;

  /*!
   * @return True if initialization is successful, otherwise False
   */
  bool initialize(const std::vector<std::string>& variable_names,
                  lwr_lib::LWRParamPtr lwr_parameters,
                  const double k_gain,
                  const double d_gain);

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
  ICRA2009DMPParamPtr parameters_;

  /*!
   */
  ICRA2009DMPStatePtr state_;

  /*!
   */
  std::vector<ICRA2009TSPtr> transformation_systems_;

  /*!
   */
  ICRA2009CSPtr canonical_system_;

};

/*! Abbreviation for convinience
 */
typedef ICRA2009DynamicMovementPrimitive ICRA2009DMP;
typedef boost::shared_ptr<ICRA2009DMP> ICRA2009DMPPtr;
typedef boost::shared_ptr<ICRA2009DMP const> ICRA2009DMPConstPtr;

}

#endif /* ICRA2009_DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_ */
