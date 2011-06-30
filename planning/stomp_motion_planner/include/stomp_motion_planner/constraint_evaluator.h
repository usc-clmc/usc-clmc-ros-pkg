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

#ifndef CONSTRAINT_EVALUATOR_H_
#define CONSTRAINT_EVALUATOR_H_

#include <stomp_motion_planner/stomp_trajectory.h>
#include <motion_planning_msgs/OrientationConstraint.h>

namespace stomp_motion_planner
{

class ConstraintEvaluator
{
public:
  ConstraintEvaluator();
  virtual ~ConstraintEvaluator();

  virtual bool getCost(const std::vector<KDL::Frame>& frame, const Eigen::VectorXd& full_trajectory_joint_pos, double& cost)=0;
};

class OrientationConstraintEvaluator: public ConstraintEvaluator
{
public:
  OrientationConstraintEvaluator(const motion_planning_msgs::OrientationConstraint &oc,
                                 const StompRobotModel& robot_model);
  virtual ~OrientationConstraintEvaluator();
  bool getCost(const std::vector<KDL::Frame>& frame, const Eigen::VectorXd& full_trajectory_joint_pos, double& cost);

private:
  double absolute_roll_tolerance_, absolute_pitch_tolerance_, absolute_yaw_tolerance_;
  double roll_weight_, pitch_weight_, yaw_weight_;
  btMatrix3x3 nominal_orientation_, nominal_orientation_inverse_;
  bool body_fixed_orientation_constraint_;
  int frame_number_;
  double weight_;
  void getRPYDistance(const btMatrix3x3 &orientation_matrix, double &roll, double &pitch, double &yaw) const;
};


}

#endif /* CONSTRAINT_EVALUATOR_H_ */
