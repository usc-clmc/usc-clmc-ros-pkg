/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef STOMP_TRAJECTORY_H_
#define STOMP_TRAJECTORY_H_

#include <trajectory_msgs/JointTrajectory.h>

#include <stomp_motion_planner/stomp_robot_model.h>
#include <stomp_motion_planner/stomp_utils.h>

#include <vector>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>

namespace stomp_motion_planner
{

/**
 * \brief Represents a discretized joint-space trajectory for STOMP
 */
class StompTrajectory
{
public:
  /**
   * \brief Constructs a trajectory for a given robot model, trajectory duration, and discretization
   */
  StompTrajectory(const StompRobotModel* robot_model, double duration, double discretization);

  /**
   * \brief Constructs a trajectory for a given robot model, number of trajectory points, and discretization
   */
  StompTrajectory(const StompRobotModel* robot_model, int num_points, double discretization);

  /**
   * \brief Creates a new containing only the joints of interest, and adds padding to the start
   * and end if needed, to have enough trajectory points for the differentiation rules
   */
  StompTrajectory(const StompTrajectory& source_traj, const StompRobotModel::StompPlanningGroup* planning_group, int diff_rule_length);

  StompTrajectory(const StompRobotModel* robot_model,
                  const StompRobotModel::StompPlanningGroup* planning_group, 
                  const trajectory_msgs::JointTrajectory& traj);

  /**
   * \brief Destructor
   */
  virtual ~StompTrajectory();

  double& operator() (int traj_point, int joint);

  double operator() (int traj_point, int joint) const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);

  void getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const;

  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);

  void overwriteTrajectory(const trajectory_msgs::JointTrajectory& traj);

  /**
   * \brief Gets the number of points in the trajectory
   */
  int getNumPoints() const;

  /**
   * \brief Gets the number of points (that are free to be optimized) in the trajectory
   */
  int getNumFreePoints() const;

  /**
   * \brief Gets the number of joints in each trajectory point
   */
  int getNumJoints() const;

  /**
   * \brief Gets the discretization time interval of the trajectory
   */
  double getDiscretization() const;

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive.
   */
  void fillInMinJerk();

  /**
   * \brief Sets the start and end index for the modifiable part of the trajectory
   *
   * (Everything before the start and after the end index is considered fixed)
   * The values default to 1 and getNumPoints()-2
   */
  void setStartEndIndex(int start_index, int end_index);

  /**
   * \brief Gets the start index
   */
  int getStartIndex() const;

  /**
   * \brief Gets the end index
   */
  int getEndIndex() const;

  /**
   * \brief Gets the trajectory duration
   */
  double getDuration() const;


  /**
   * \brief Gets the entire trajectory matrix
   */
  Eigen::MatrixXd& getTrajectory();

  /**
   * \brief Gets the block of the trajectory which can be optimized
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>  getFreeTrajectoryBlock();

  /**
   * \brief Gets the block of free (optimizable) trajectory for a single joint
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(int joint);

  /**
   * \brief Updates the full trajectory (*this) from the group trajectory
   */
  void updateFromGroupTrajectory(const StompTrajectory& group_trajectory);

  /**
   * \brief Gets the index in the full trajectory which was copied to this group trajectory
   */
  int getFullTrajectoryIndex(int i) const;

  /**
   * \brief Gets the joint velocities at the given trajectory point
   */
  template <typename Derived>
  void getJointVelocities(int traj_point, Eigen::MatrixBase<Derived>& velocities);

  /**
   * \brief Gets the joint accelerations at the given trajectory point
   */
  template <typename Derived>
  void getJointAccelerations(int traj_point, Eigen::MatrixBase<Derived>& accelerations);

private:

  void init();                                          /**< \brief Allocates memory for the trajectory */

  const StompRobotModel* robot_model_;                  /**< Robot Model */
  const StompRobotModel::StompPlanningGroup* planning_group_;    /**< Planning group that this trajectory corresponds to, if any */
  int num_points_;                                      /**< Number of points in the trajectory */
  int num_joints_;                                      /**< Number of joints in each trajectory point */
  double discretization_;                               /**< Discretization of the trajectory */
  double duration_;                                     /**< Duration of the trajectory */
  Eigen::MatrixXd trajectory_;                          /**< Storage for the actual trajectory */
  int start_index_;                                     /**< Start index (inclusive) of trajectory to be optimized (everything before it will not be modified) */
  int end_index_;                                       /**< End index (inclusive) of trajectory to be optimized (everything after it will not be modified) */
  std::vector<int> full_trajectory_index_;              /**< If this is a "group" trajectory, the index from the original traj which each element here was copied */
};

///////////////////////// inline functions follow //////////////////////

inline double& StompTrajectory::operator() (int traj_point, int joint)
{
  return trajectory_(traj_point, joint);
}

inline double StompTrajectory::operator() (int traj_point, int joint) const
{
  return trajectory_(traj_point, joint);
}

inline Eigen::MatrixXd::RowXpr StompTrajectory::getTrajectoryPoint(int traj_point)
{
  return trajectory_.row(traj_point);
}

inline Eigen::MatrixXd::ColXpr StompTrajectory::getJointTrajectory(int joint)
{
  return trajectory_.col(joint);
}

inline int StompTrajectory::getNumPoints() const
{
  return num_points_;
}

inline int StompTrajectory::getNumFreePoints() const
{
  return (end_index_ - start_index_)+1;
}

inline int StompTrajectory::getNumJoints() const
{
  return num_joints_;
}

inline double StompTrajectory::getDiscretization() const
{
  return discretization_;
}

inline void StompTrajectory::setStartEndIndex(int start_index, int end_index)
{
  start_index_ = start_index;
  end_index_ = end_index;
}

inline int StompTrajectory::getStartIndex() const
{
  return start_index_;
}

inline int StompTrajectory::getEndIndex() const
{
  return end_index_;
}

inline double StompTrajectory::getDuration() const
{
  return duration_;
}

inline Eigen::MatrixXd& StompTrajectory::getTrajectory()
{
  return trajectory_;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> StompTrajectory::getFreeTrajectoryBlock()
{
  return trajectory_.block(start_index_, 0, getNumFreePoints(), getNumJoints());
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> StompTrajectory::getFreeJointTrajectoryBlock(int joint)
{
  return trajectory_.block(start_index_, joint, getNumFreePoints(), 1);
}

inline void StompTrajectory::getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const
{
  for (int i=0; i<num_joints_; i++)
    kdl_jnt_array(i) = trajectory_(traj_point,i);
}

inline int StompTrajectory::getFullTrajectoryIndex(int i) const
{
  return full_trajectory_index_[i];
}

template <typename Derived>
void StompTrajectory::getJointVelocities(int traj_point, Eigen::MatrixBase<Derived>& velocities)
{
  velocities.setZero();
  double invTime = 1.0 / discretization_;
  //printf("invTime = %lf\n", invTime);

  for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
  {
    velocities += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) * trajectory_.row(traj_point+k).transpose();
  }
}

template <typename Derived>
void StompTrajectory::getJointAccelerations(int traj_point, Eigen::MatrixBase<Derived>& accelerations)
{
  accelerations.setZero();
  double invTime2 = 1.0 / (discretization_*discretization_);

  //printf("invTime2 = %lf\n", invTime2);
  for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
  {
    accelerations += (invTime2 * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) * trajectory_.row(traj_point+k).transpose();
  }
}

}

#endif /* STOMP_TRAJECTORY_H_ */
