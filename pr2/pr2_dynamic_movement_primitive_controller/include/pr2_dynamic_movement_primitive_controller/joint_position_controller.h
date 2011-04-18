/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef JOINT_POSITION_CONTROLLER_H
#define JOINT_POSITION_CONTROLLER_H

/**
@class pr2_controller_interface::JointPositionController
@brief Joint Position Controller

This class controls positon using a pid loop.

@section ROS ROS interface

@param type Must be "JointPositionController"
@param joint Name of the joint to control.
@param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

Subscribes to:

- @b command (std_msgs::Float64) : The joint position to achieve.

Publishes:

- @b state (robot_mechanism_controllers::JointControllerState) :
Current state of the controller, including pid error and gains.

*/

// ros includes
#include <ros/node_handle.h>
#include <rosrt/rosrt.h>

#include <pr2_controller_interface/controller.h>

#include <boost/shared_ptr.hpp>

#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>

#include <pr2_controllers_msgs/JointControllerState.h>

#include <angles/angles.h>

namespace pr2_dynamic_movement_primitive_controller
{

class JointPositionController // : public pr2_controller_interface::Controller
{
public:

    /*!
    * @return
    */
    JointPositionController();
    ~JointPositionController();

    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node_handle);

    /*!
    * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
    *
    * \param command
    */
    void setCommand(const double cmd);

    /*!
    * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
    */
    void getCommand(double & cmd);

    /*!
    *
    */
    void starting();

    /*!
    * \brief Issues commands to the joint. Should be called at regular intervals
    */
    void update();

    void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

    std::string getJointName();

    double getJointPosition() const;
    double getJointVelocity() const;

    double getJointPositionError() const;
    double getJointVelocityError() const;

    double getCommandedEffort() const;

    ros::Duration dt_;

private:

    bool initialized_;

    pr2_mechanism_model::JointState *joint_state_;        /**< Joint we're controlling. */

    double command_;                                      /**< Last commanded position. */
    double commanded_effort_;                             /**< Last commanded effort. */

    int publisher_rate_;
    int publisher_counter_;
    int publisher_buffer_size_;

    double error_;
    double error_dot_;
    double last_error_;

    pr2_mechanism_model::RobotState *robot_;              /**< Pointer to robot structure. */
    control_toolbox::Pid pid_controller_;                 /**< Internal PID controller. */

    ros::NodeHandle node_handle_;

    ros::Time last_time_;                                 /**< Last time stamp of update. */

    boost::shared_ptr<rosrt::Publisher<pr2_controllers_msgs::JointControllerState> > controller_state_publisher_;

    void publish();

};

inline double JointPositionController::getJointPosition() const
{
    if (joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
    {
        return angles::normalize_angle(joint_state_->position_);
    }
    return joint_state_->position_;
}
inline double JointPositionController::getJointVelocity() const
{
    return joint_state_->velocity_;
}

inline double JointPositionController::getJointPositionError() const
{
    return error_;
}
inline double JointPositionController::getJointVelocityError() const
{
    return error_dot_;
}

inline double JointPositionController::getCommandedEffort() const
{
    return commanded_effort_;
}


} // namespace

#endif
