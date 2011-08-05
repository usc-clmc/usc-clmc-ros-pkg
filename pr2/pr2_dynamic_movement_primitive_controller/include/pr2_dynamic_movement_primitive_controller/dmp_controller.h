/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   dmp_controller.h

  \author Alexander Herzog, Peter Pastor
  \date   Mar 1, 2011

 *********************************************************************/

#ifndef DMP_CONTROLLER_H_
#define DMP_CONTROLLER_H_

#include <rosrt/rosrt.h>

#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/ControllerStatusMsg.h>

#include <pr2_dynamic_movement_primitive_controller/variable_name_map.h>

// local includes

namespace pr2_dynamic_movement_primitive_controller
{

class DMPController
{
public:

  /*! Constructor
   */
  DMPController() :
    initialized_(false) {};
  virtual ~DMPController() {};

  /*!
   * @param name
   * @return True on success, otherwise False
   */
  virtual bool initialize(const std::string& controller_name,
                          const std::vector<std::string>& variable_names) = 0;

  /*!
   * @param new_start
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  virtual bool changeDMPStart(const Eigen::VectorXd& new_start) = 0;

  /*!
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  virtual bool newDMPReady() = 0;

  /*!
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  virtual bool isRunning(Eigen::VectorXd& desired_positions,
                         Eigen::VectorXd& desired_velocities,
                         Eigen::VectorXd& desired_accelerations) = 0;

  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  virtual bool getDMP(dmp_lib::DMPPtr& dmp) = 0;

  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  virtual bool stop() = 0;

  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  const VariableNameMap& getVariableNameMap() const
  {
    return variable_name_map_;
  }
  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  bool getNumUsedVariables(int& num_variables_used) const
  {
    if(dmp_is_set_)
    {
      num_variables_used = num_variables_used_;
      return true;
    }
    return false;
  }

  /*!
   * @return
   * REAL-TIME REQUIREMENTS
   */
  bool isIdle()
  {
    return !dmp_is_being_executed_;
  }

protected:

  /*!
   */
  bool initialized_;

  /*!
   */
  bool dmp_is_being_executed_;
  bool dmp_is_set_;

  /*!
   */
  VariableNameMap variable_name_map_;

  /*!
   */
  int num_variables_used_;
  Eigen::VectorXd entire_desired_positions_;
  Eigen::VectorXd entire_desired_velocities_;
  Eigen::VectorXd entire_desired_accelerations_;

  /*!
   */
  rosrt::Publisher<dynamic_movement_primitive::ControllerStatusMsg> dmp_status_publisher_;

  /*!
   */
  ros::Time start_time_;

};

}

#endif /* DMP_CONTROLLER_H_ */
