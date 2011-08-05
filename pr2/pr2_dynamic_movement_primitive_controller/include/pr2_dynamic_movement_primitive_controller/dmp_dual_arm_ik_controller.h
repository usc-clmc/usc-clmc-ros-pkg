/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \file    dmp_dual_arm_ik_controller.h

 \author  Peter Pastor
 \date    Jul 26, 2010

 **********************************************************************/

#ifndef DMP_DUAL_ARM_IK_CONTROLLER_H_
#define DMP_DUAL_ARM_IK_CONTROLLER_H_

// system includes
#include <pthread.h>

// ros includes
#include <ros/ros.h>
#include <rosrt/rosrt.h>
#include <pr2_controller_interface/controller.h>

#include <boost/circular_buffer.hpp>

// local includes
#include <pr2_dynamic_movement_primitive_controller/dmp_dual_arm_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_ik_controller.h>

namespace pr2_dynamic_movement_primitive_controller
{

class DMPDualArmIkController : public pr2_controller_interface::Controller
{

  typedef DMPIkController ChildController;

public:

  /*!
   * @return
   */
  DMPDualArmIkController();
  virtual ~DMPDualArmIkController() {};

  /*!
   * @param robot_state
   * @param node_handle
   * @return
   */
  bool init(pr2_mechanism_model::RobotState* robot_state,
            ros::NodeHandle& node_handle);

  /*!
   */
  void starting();

  /*!
   */
  void update();

  /*!
   */
  void stopping();

  /*!
   * @return
   */
  bool initXml(pr2_mechanism_model::RobotState* robot,
               TiXmlElement* config);

//  /*!
//   * @param request
//   * @param response
//   * @return
//   */
//  bool addToExecuteDMPQueue(dmp_motion_controller::AddToDualArmExecuteDMPQueue::Request& request,
//                            dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response& response);

private:

  bool initialized_;
  ros::NodeHandle node_handle_;

  ChildController* left_ik_controller_;
  ChildController* right_ik_controller_;

  // DMPController
  boost::shared_ptr<DMPController> dmp_controller_;

//  /*!
//   * @param dmps
//   * @param arm_types
//   * @param extracted_dmps
//   * @return
//   */
//  bool extractDMP(const std::vector<dmp_motion_generation::DynamicMovementPrimitive>& dmps,
//                  const std::vector<int>& arm_types,
//                  std::vector<dmp_motion_generation::DynamicMovementPrimitive>& extracted_dmps);

  /*!
   * @return
   */
  bool getChildController();

};

}

#endif /* DMP_DUAL_ARM_IK_CONTROLLER_H_ */
