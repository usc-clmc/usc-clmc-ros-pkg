/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <stomp_motion_planner/constraint_evaluator.h>

namespace stomp_motion_planner
{

ConstraintEvaluator::ConstraintEvaluator()
{
}

ConstraintEvaluator::~ConstraintEvaluator()
{
}

OrientationConstraintEvaluator::OrientationConstraintEvaluator(const motion_planning_msgs::OrientationConstraint &oc,
                                                               const StompRobotModel& robot_model)
{
  frame_number_ = robot_model.getForwardKinematicsSolver()->segmentNameToIndex(oc.link_name);
  btQuaternion q;
  tf::quaternionMsgToTF(oc.orientation,q);
  nominal_orientation_ = btMatrix3x3(q);
  nominal_orientation_inverse_ = nominal_orientation_.inverse();
  if(oc.type == oc.HEADER_FRAME)
    body_fixed_orientation_constraint_ = false;
  else
    body_fixed_orientation_constraint_ = true;
  absolute_roll_tolerance_ = oc.absolute_roll_tolerance;
  absolute_pitch_tolerance_ = oc.absolute_pitch_tolerance;
  absolute_yaw_tolerance_ = oc.absolute_yaw_tolerance;
  weight_ = oc.weight;

  roll_weight_ = pitch_weight_ = yaw_weight_ = 1.0;
  if (absolute_pitch_tolerance_ >= M_PI)
    pitch_weight_ = 0.0;
  if (absolute_roll_tolerance_ >= M_PI)
    roll_weight_ = 0.0;
  if (absolute_yaw_tolerance_ >= M_PI)
    yaw_weight_ = 0.0;
}

OrientationConstraintEvaluator::~OrientationConstraintEvaluator()
{
}

bool OrientationConstraintEvaluator::getCost(const std::vector<KDL::Frame>& frame, const Eigen::VectorXd& full_trajectory_joint_pos, double& cost)
{
  double w, x, y, z;
  frame[frame_number_].M.GetQuaternion(x, y, z, w);
  btMatrix3x3 orientation_matrix(btQuaternion(x, y, z, w));

  double roll, pitch, yaw;
  getRPYDistance(orientation_matrix, roll, pitch, yaw);

  roll=fabs(roll);
  pitch=fabs(pitch);
  yaw=fabs(yaw);

  bool satisfied = true;
  cost = weight_ * (roll_weight_ * roll + pitch_weight_*pitch + yaw_weight_*yaw);

  if (roll > absolute_roll_tolerance_ || pitch > absolute_pitch_tolerance_ || yaw > absolute_yaw_tolerance_)
    satisfied = false;

  return satisfied;
}

void OrientationConstraintEvaluator::getRPYDistance(const btMatrix3x3 &orientation_matrix, double &roll, double &pitch, double &yaw) const
{
  if(!body_fixed_orientation_constraint_)
  {
    btMatrix3x3 result = orientation_matrix * nominal_orientation_inverse_;
    result.getRPY(roll, pitch, yaw);
  }
  else
  {
    btMatrix3x3 result = nominal_orientation_inverse_ * orientation_matrix;
    result.getRPY(roll, pitch, yaw);
  }
}


}
