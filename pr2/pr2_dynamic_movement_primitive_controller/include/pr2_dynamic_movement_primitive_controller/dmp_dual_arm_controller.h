/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   dmp_dual_arm_controller.h

  \author Peter Pastor
  \date   Mar 1, 2011

 *********************************************************************/

#ifndef DMP_DUAL_ARM_CONTROLLER_H_
#define DMP_DUAL_ARM_CONTROLLER_H_

#include <rosrt/rosrt.h>

#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/ControllerStatusMsg.h>

// local includes

namespace pr2_dynamic_movement_primitive_controller
{

class DMPDualArmController
{
public:

  /*! Constructor
   */
  DMPDualArmController() :
    initialized_(false) {};
  virtual ~DMPDualArmController() {};

  /*!
   * @param controller_name
   * @return True on success, otherwise False
   */
  virtual bool initialize(const std::string& controller_name) = 0;

  /*!
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  virtual bool newDMPReady() = 0;

protected:

  /*!
   */
  bool initialized_;

  /*!
   */
  rosrt::Publisher<dynamic_movement_primitive::ControllerStatusMsg> dmp_status_publisher_;

  /*!
   */
  ros::Time start_time_;

};

}

#endif /* DMP_DUAL_ARM_CONTROLLER_H_ */
