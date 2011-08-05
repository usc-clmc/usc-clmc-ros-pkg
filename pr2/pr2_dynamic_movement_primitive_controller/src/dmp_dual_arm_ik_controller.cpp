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

 \file    DMPDualArmIkController.cpp

 \author  Peter Pastor
 \date    Jul 26, 2010

 **********************************************************************/

// system includes

// ros includes
#include <pluginlib/class_list_macros.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// #include <pr2_tasks_transforms/task_transforms.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/dmp_dual_arm_ik_controller.h>

PLUGINLIB_DECLARE_CLASS(pr2_dynamic_movement_primitive_controller, DMPDualArmIkController, pr2_dynamic_movement_primitive_controller::DMPDualArmIkController, pr2_controller_interface::Controller)

// using namespace pr2_tasks_transforms;

namespace pr2_dynamic_movement_primitive_controller
{

const int SIZE_RUN_QUEUE = 100;

DMPDualArmIkController::DMPDualArmIkController() :
  initialized_(false)
{
}

bool DMPDualArmIkController::init(pr2_mechanism_model::RobotState* robot_state,
                                  ros::NodeHandle& node_handle)
{
  node_handle_ = node_handle;
  assert(robot_state);



  // get pointer to left and right ik controllers
  ROS_VERIFY(getChildController());

  return (initialized_ = true);
}

bool DMPDualArmIkController::getChildController()
{
  // get a pointer to the ik controllers
  std::string left_arm_controller_name;
  std::string right_arm_controller_name;
  ROS_VERIFY(usc_utilities::read(node_handle_, std::string("left_arm_controller"), left_arm_controller_name));
  ROS_VERIFY(usc_utilities::read(node_handle_, std::string("right_arm_controller"), right_arm_controller_name));

  ros::Duration timeout(20);
  ros::Time start_time = ros::Time::now();
  bool found_controller = false;
  do
  {
    found_controller = getController<ChildController> (left_arm_controller_name, AFTER_ME, left_ik_controller_);
    if (!found_controller)
    {
      ros::Duration(0.5).sleep();
    }
  } while (!found_controller && ((start_time + timeout) > ros::Time::now()));
  if (!found_controller)
  {
    ROS_ERROR("Could not connect to left ik controller \"%s\"", left_arm_controller_name.c_str());
    return false;
  }

  start_time = ros::Time::now();
  found_controller = false;
  do
  {
    found_controller = getController<ChildController> (right_arm_controller_name, AFTER_ME, right_ik_controller_);
    if (!found_controller)
    {
      ros::Duration(0.5).sleep();
    }
  } while (!found_controller && ((start_time + timeout) > ros::Time::now()));
  if (!found_controller)
  {
    ROS_ERROR("Could not connect to right ik controller \"%s\"", right_arm_controller_name.c_str());
    return false;
  }
  return true;
}

void DMPDualArmIkController::starting()
{
}

void DMPDualArmIkController::update()
{
}

void DMPDualArmIkController::stopping()
{
}

bool DMPDualArmIkController::initXml(pr2_mechanism_model::RobotState* robot,
                                     TiXmlElement* config)
{
  ros::NodeHandle node_handle(config->Attribute("name"));
  return init(robot, node_handle);
}

/*
bool DMPDualArmIkController::addToExecuteDMPQueue(dmp_motion_controller::AddToDualArmExecuteDMPQueue::Request& request,
                                                  dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response& response)
{
  if ((request.left_arm_dmps.size() != request.left_arm_types.size()) || (request.right_arm_dmps.size() != request.right_arm_types.size())
      || (request.execution_durations.size() != request.right_arm_types.size()))
  {
    response.info.assign("Number of dmps, transforms, and execution_durations does not match.");
    response.return_code = dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
    return true;
  }

  if (pthread_mutex_lock(&dmp_cmd_lock_) == 0)
  {
    for (int i = 0; i < static_cast<int> (request.execution_durations.size()); ++i)
    {

      dmp_motion_controller::AddToExecuteDMPQueue::Request left_arm_request;
      dmp_motion_controller::AddToExecuteDMPQueue::Request right_arm_request;

      //std::vector<dmp_motion_generation::DynamicMovementPrimitive> left_arm_dmps;
      if (!extractDMP(request.left_arm_dmps, request.left_arm_types, left_arm_request.dmps))
      {
        ROS_ERROR("Could not extract left arm dmps.");
        response.info.assign("Could not extract left arm dmps.");
        response.return_code = dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
        return true;
      }
      //left_arm_request.dmps = left_arm_dmps;
      left_arm_request.execution_durations = request.execution_durations;
      left_arm_request.types = request.left_arm_types;

      //std::vector<dmp_motion_generation::DynamicMovementPrimitive> right_arm_dmps;
      if (!extractDMP(request.right_arm_dmps, request.right_arm_types, right_arm_request.dmps))
      {
        ROS_ERROR("Could not extract right arm dmps.");
        response.info.assign("Could not extract right arm dmps.");
        response.return_code = dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
        return true;
      }
      //right_arm_request.dmps = right_arm_dmps;
      right_arm_request.execution_durations = request.execution_durations;
      right_arm_request.types = request.right_arm_types;

      dmp_motion_controller::AddToExecuteDMPQueue::Response left_arm_response;
      dmp_motion_controller::AddToExecuteDMPQueue::Response right_arm_response;

      DualArmDMPStruct dual_arm_dmp_struct;
      ros::NodeHandle right_arm_node_handle("/r_arm_dmp_ik_controller");
      if (!DMPControllerCommon::setDMPStructFromRequest(right_arm_node_handle, right_arm_request, right_arm_response, right_ik_controller_->getNumDofs(),
                                                        right_ik_controller_->getVariableNamesKeyWord(), dual_arm_dmp_struct.right_arm_dmp_structs))
      {
        // setting dmp_structs failed...
        return true;
      }
      ros::NodeHandle left_arm_node_handle("/l_arm_dmp_ik_controller");
      if (!DMPControllerCommon::setDMPStructFromRequest(left_arm_node_handle, left_arm_request, left_arm_response, left_ik_controller_->getNumDofs(),
                                                        left_ik_controller_->getVariableNamesKeyWord(), dual_arm_dmp_struct.left_arm_dmp_structs))
      {
        // setting dmp_structs failed...
        return true;
      }

      dual_arm_dmp_buffer_.push_back(dual_arm_dmp_struct);
    }
    pthread_mutex_unlock(&dmp_cmd_lock_);
  }

  //    DualArmRequestStruct dual_arm_struct;
  //    dual_arm_struct.request = request;
  //    dual_arm_struct.response = response;
  //
  //    if (pthread_mutex_lock(&dmp_cmd_lock_) == 0)
  //    {
  //        dual_arm_request_buffer_.push_back(dual_arm_struct);
  //        pthread_mutex_unlock(&dmp_cmd_lock_);
  //    }

  return true;
}
*/

/*
bool DMPDualArmIkController::extractDMP(const std::vector<dmp_motion_generation::DynamicMovementPrimitive>& dmps,
                                        const std::vector<int>& arm_types,
                                        std::vector<dmp_motion_generation::DynamicMovementPrimitive>& extracted_dmps)
{

  // TODO: change this horrible horrible hack.
  const unsigned int num_no_transform_dim = dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS;
  const unsigned int num_left_arm_pool_transform_dim = num_no_transform_dim;
  const unsigned int num_right_arm_pool_transform_dim = 4 + dmp::N_JOINTS;

  const unsigned int num_right_arm_chop_stick_transform_dim = dmp::N_CART + dmp::N_CART + dmp::N_JOINTS;
  const unsigned int num_left_arm_chop_stick_transform_dim = dmp::N_CART + dmp::N_CART + dmp::N_JOINTS;

  if (dmps.size() != arm_types.size())
  {
    ROS_ERROR("Wrong number of dmps and arm types.");
    return false;
  }

  // TODO: check whether real-time performace is needed here.
  extracted_dmps.clear();
  for (int i = 0; i < static_cast<int> (dmps.size()); ++i)
  {
    TaskTransforms::TransformType arm_type = static_cast<TaskTransforms::TransformType> (arm_types[i]);
    dmp_motion_generation::DynamicMovementPrimitive extracted_dmp = dmps[i];
    switch (arm_type)
    {
      case TaskTransforms::LEFT_ARM_NO_TRANSFORM:
      {
        if (dmps[i].transformation_systems.size() != num_no_transform_dim)
        {
          extracted_dmp.transformation_systems.erase(extracted_dmp.transformation_systems.begin(), extracted_dmp.transformation_systems.end()
              - num_no_transform_dim);
        }
        break;
      }
      case TaskTransforms::RIGHT_ARM_NO_TRANSFORM:
      {
        if (dmps[i].transformation_systems.size() != num_no_transform_dim)
        {
          extracted_dmp.transformation_systems.clear();
          extracted_dmp.transformation_systems.insert(extracted_dmp.transformation_systems.begin(), dmps[i].transformation_systems.begin(),
                                                      dmps[i].transformation_systems.begin() + num_no_transform_dim);
        }
        break;
      }
      case TaskTransforms::LEFT_ARM_POOL_TRANSFORM:
      {
        if (dmps[i].transformation_systems.size() != num_left_arm_pool_transform_dim)
        {
          extracted_dmp.transformation_systems.erase(extracted_dmp.transformation_systems.begin(), extracted_dmp.transformation_systems.end()
              - num_left_arm_pool_transform_dim);
        }
        break;
      }
      case TaskTransforms::RIGHT_ARM_POOL_TRANSFORM:
      {
        if (dmps[i].transformation_systems.size() != num_right_arm_pool_transform_dim)
        {
          extracted_dmp.transformation_systems.clear();
          extracted_dmp.transformation_systems.insert(extracted_dmp.transformation_systems.begin(), dmps[i].transformation_systems.begin(),
                                                      dmps[i].transformation_systems.begin() + num_right_arm_pool_transform_dim);
        }
        break;
      }
      case TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM:
      {
        if (dmps[i].transformation_systems.size() != num_left_arm_chop_stick_transform_dim)
        {
          extracted_dmp.transformation_systems.erase(extracted_dmp.transformation_systems.begin(), extracted_dmp.transformation_systems.end()
              - num_left_arm_chop_stick_transform_dim);
        }
        break;
      }
      case TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM:
      {
        if (dmps[i].transformation_systems.size() != num_right_arm_chop_stick_transform_dim)
        {
          extracted_dmp.transformation_systems.clear();
          extracted_dmp.transformation_systems.insert(extracted_dmp.transformation_systems.begin(), dmps[i].transformation_systems.begin(),
                                                      dmps[i].transformation_systems.begin() + num_right_arm_chop_stick_transform_dim);
        }
        break;
      }
      default:
      {
        ROS_ERROR("Invalid arm type (%i) received.", static_cast<int> (arm_types[i]));
        return false;
      }
    }
    extracted_dmps.push_back(extracted_dmp);
  }
  return true;
}
*/

}
