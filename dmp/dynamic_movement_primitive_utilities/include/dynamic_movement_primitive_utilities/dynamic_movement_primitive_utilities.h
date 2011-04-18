/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_utilities.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 22, 2011

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_UTILITIES_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_UTILITIES_H_

// system includes
#include <vector>

#include <ros/ros.h>

#include <inverse_kinematics/inverse_kinematics_with_nullspace_optimization.h>

#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/DMPUtilitiesMsg.h>
#include <dynamic_movement_primitive/TypeMsg.h>

// local includes

namespace dmp_utilities
{

class DynamicMovementPrimitiveUtilities
{

public:

  /*!
   *
   * @param dmp
   * @param controller_name
   * @return True on success, otherwise False
   */
  static bool getControllerName(const dmp_lib::DMPPtr dmp,
                                std::string& controller_name);

  /*!
   * @param dmp
   * @param cost
   * @return True on success, otherwise False
   */
  static bool computeCost(const dmp_lib::DMPPtr dmp,
                          double& cost);

  /*!
   * @param dmp_utilities_msg
   * @param dmp
   * @return True on success, otherwise False
   */
  static bool getDMP(const dynamic_movement_primitive::DMPUtilitiesMsg& dmp_utilities_msg,
                     dmp_lib::DMPPtr& dmp);

  /*!
   * @param msg
   * @param dmp
   * @return True on success, otherwise False
   */
  static bool getDMP(const dmp::ICRA2009DMPMsg& msg,
                     dmp_lib::DMPPtr& dmp);

  /*!
   * @param msg
   * @param dmp
   * @return True on success, otherwise False
   */
  static bool getDMP(const dmp::NC2010DMPMsg& msg,
                     dmp_lib::DMPPtr& dmp);


  /*! Writes the dmp to file
   * @param abs_bag_file_name
   * @param dmp
   * @return True on success, otherwise False
   */
  static bool writeToFile(const std::string& abs_bag_file_name, const dmp_lib::DMPPtr& dmp);

  /*! Writes the dmp to file
   * @param abs_bag_file_name
   * @param dmp
   * @return True on success, otherwise False
   */
  static bool readFromFile(const std::string& abs_bag_file_name, dmp_lib::DMPPtr& dmp);

  /*!
   * @param dmp
   * @param msg
   * @return True on success, otherwise False
   */
  static bool setMsg(const dmp_lib::DMPPtr& dmp,
                     dmp::ICRA2009DMPMsg& msg);

  /*!
   * @param dmp
   * @param msg
   * @return True on success, otherwise False
   */
  static bool setMsg(const dmp_lib::DMPPtr& dmp,
                     dmp::NC2010DMPMsg& msg);

  /*!
   * @param msg
   * @param goal
   * @return True on success, otherwise False
   */
  static bool getGoal(const dmp::ICRA2009DMPMsg& msg,
                      std::vector<double> goal);

  /*!
   * @param msg
   * @param goal
   * @return True on success, otherwise False
   */
  static bool getGoal(const dmp::NC2010DMPMsg& msg,
                      std::vector<double> goal);

  /*!
   * @param msg
   * @param variable_names
   * @param goal
   * @return True on success, otherwise False
   */
  static bool getGoal(const dmp::ICRA2009DMPMsg& msg,
                      const std::vector<std::string>& variable_names,
                      std::vector<double> goal);

  /*!
   * @param msg
   * @param variable_names
   * @param goal
   * @return True on success, otherwise False
   */
  static bool getGoal(const dmp::NC2010DMPMsg& msg,
                      const std::vector<std::string>& variable_names,
                      std::vector<double> goal);

private:

  /*! Constructor
   */
  DynamicMovementPrimitiveUtilities() {};

  /*! Destructor
   */
  virtual ~DynamicMovementPrimitiveUtilities() {};

};

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_UTILITIES_H_ */
