/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		nc2010_dynamic_movement_primitive.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 22, 2010

 *********************************************************************/

#ifndef NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_
#define NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_

// system includes
#include <vector>
#include <string>

// #include <lwr_lib/lwr_parameters.h>

// local includes
#include <dmp_lib/dynamic_movement_primitive.h>
#include <dmp_lib/nc2010_dynamic_movement_primitive_state.h>
#include <dmp_lib/nc2010_dynamic_movement_primitive_parameters.h>
#include <dmp_lib/nc2010_transformation_system.h>
#include <dmp_lib/nc2010_canonical_system.h>

namespace dmp_lib
{

class NC2010DynamicMovementPrimitive : public DynamicMovementPrimitive
{

public:

  /*! Constructor
   */
  NC2010DynamicMovementPrimitive() {};

  /*! Destructor
   */
  virtual ~NC2010DynamicMovementPrimitive() {};

  /*! Assignment operator
   * @param nc2010dmp
   * @return
   */
  NC2010DynamicMovementPrimitive& operator=(const NC2010DynamicMovementPrimitive& nc2010dmp);

  /*!
   * @param parameters
   * @param state
   * @param transformation_systems
   * @param canonical_system
   * @return True on success, otherwise False
   */
  bool initialize(NC2010DMPParamPtr& parameters,
                  NC2010DMPStatePtr& state,
                  std::vector<NC2010TSPtr>& transformation_systems,
                  NC2010CSPtr& canonical_system);

  /*!
   * @param nc2010_dmp
   * @return
   */
  bool add(const NC2010DynamicMovementPrimitive& nc2010_dmp, bool check_for_compatibiliy = true);

  /*!
   * @param parameters
   * @param state
   * @return True on success, otherwise False
   */
  bool get(NC2010DMPParamConstPtr& parameters,
           NC2010DMPStateConstPtr& state,
           std::vector<NC2010TSConstPtr>& transformation_systems,
           NC2010CSConstPtr& canonical_system) const;

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
    return "NC2010";
  }

private:

  /*!
   */
  NC2010DMPParamPtr parameters_;

  /*!
   */
  NC2010DMPStatePtr state_;

  /*!
   */
  std::vector<NC2010TSPtr> transformation_systems_;

  /*!
   */
  NC2010CSPtr canonical_system_;

};

/*! Abbreviation for convinience
 */
typedef NC2010DynamicMovementPrimitive NC2010DMP;
typedef boost::shared_ptr<NC2010DMP> NC2010DMPPtr;
typedef boost::shared_ptr<NC2010DMP const> NC2010DMPConstPtr;

}

#endif /* NC2010_DYNAMIC_MOVEMENT_PRIMITIVE_BASE_H_ */
