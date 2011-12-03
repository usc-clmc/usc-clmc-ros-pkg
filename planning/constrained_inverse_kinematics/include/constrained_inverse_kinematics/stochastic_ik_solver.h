/*
 * stochastic_ik_solver.h
 *
 *  Created on: Jul 19, 2011
 *      Author: kalakris
 */

#ifndef STOCHASTIC_IK_SOLVER_H_
#define STOCHASTIC_IK_SOLVER_H_

#include <ros/ros.h>
#include <constrained_inverse_kinematics/chain.h>
#include <queue>

namespace constrained_inverse_kinematics
{

struct SampleInfo
{
  KDL::JntArray joint_angles;
  double cost;
  double probability;
};

class StochasticIKSolver
{
public:
  StochasticIKSolver(ros::NodeHandle& node_handle, const std::string& root, const std::string& tip);
  virtual ~StochasticIKSolver();

  bool solve(const KDL::Frame& pose_des,
             const KDL::JntArray& q_in,
             KDL::JntArray& q_out);

private:
  ros::NodeHandle node_handle_;
  Chain chain_;

  std::vector<double> noise_stddev_;
  std::vector<double> noise_decay_;
  int num_samples_per_iteration_;
  int max_iterations_;

  double position_cost_weight_;
  double orientation_cost_weight_;
  double cost_to_probability_h_;

  void readParams();

  double computeCost(const KDL::JntArray& q_in, const KDL::Frame& pose_des, KinematicsInfo& kinematics_info);

};

}

#endif /* STOCHASTIC_IK_SOLVER_H_ */
