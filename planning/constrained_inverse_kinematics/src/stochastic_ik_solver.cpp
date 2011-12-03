/*
 * stochastic_ik_solver.cpp
 *
 *  Created on: Jul 19, 2011
 *      Author: kalakris
 */

#include <limits>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <constrained_inverse_kinematics/stochastic_ik_solver.h>
#include <constrained_inverse_kinematics/multivariate_gaussian.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

namespace constrained_inverse_kinematics
{

StochasticIKSolver::StochasticIKSolver(ros::NodeHandle& node_handle, const std::string& root, const std::string& tip):
    node_handle_(node_handle),
    chain_(root, tip)
{
  readParams();
}

StochasticIKSolver::~StochasticIKSolver()
{
}

bool StochasticIKSolver::solve(const KDL::Frame& pose_des,
           const KDL::JntArray& q_in,
           KDL::JntArray& q_out)
{
  MultivariateGaussian gaussian;
  KinematicsInfo kinematics_info;
  KDL::Frame pose;
  KDL::Twist error;

  SampleInfo default_sample_info;
  default_sample_info.joint_angles = q_in;
  std::vector<SampleInfo> samples;
  for (int j=0; j<num_samples_per_iteration_; ++j)
    samples.push_back(default_sample_info);

  KDL::JntArray joint_angles = q_in;
  Eigen::VectorXd prev_mean = joint_angles.data;
  Eigen::VectorXd diff = prev_mean;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(chain_.num_joints_, chain_.num_joints_);
  for (int k=0; k<chain_.num_joints_; ++k)
  {
    covariance(k,k) = noise_stddev_[k]*noise_stddev_[k];
  }

  for (int i=0; i<max_iterations_; ++i)
  {
    double cost = computeCost(joint_angles, pose_des, kinematics_info);
    ROS_INFO("Iteration %d: cost = %f", i, cost);
    pose = kinematics_info.link_frames_.back();
    error = KDL::diff(pose_des, pose);
    ROS_INFO("Position error = %f, %f, %f", error.vel.data[0], error.vel.data[1], error.vel.data[2]);

    gaussian.setMeanAndCovariance(joint_angles.data, covariance);

    for (int j=0; j<num_samples_per_iteration_; ++j)
    {
      // add noise and clip
      gaussian.sample(samples[j].joint_angles.data);
      for (int k=0; k<chain_.num_joints_; ++k)
      {
        if (samples[j].joint_angles(k) > chain_.joints_[k]->limits->upper)
          samples[j].joint_angles(k) = chain_.joints_[k]->limits->upper;
        if (samples[j].joint_angles(k) < chain_.joints_[k]->limits->lower)
          samples[j].joint_angles(k) = chain_.joints_[k]->limits->lower;
      }

      // compute cost
      samples[j].cost = computeCost(samples[j].joint_angles, pose_des, kinematics_info);
    }

    // map costs to probabilities
    double max_cost = -std::numeric_limits<double>::max();
    double min_cost = std::numeric_limits<double>::max();
    for (int j=0; j<num_samples_per_iteration_; ++j)
    {
      if (samples[j].cost > max_cost)
        max_cost = samples[j].cost;
      if (samples[j].cost < min_cost)
        min_cost = samples[j].cost;
    }
    double p_sum = 0.0;
    for (int j=0; j<num_samples_per_iteration_; ++j)
    {
      samples[j].probability = exp(-cost_to_probability_h_ * (samples[j].cost - min_cost)/(max_cost - min_cost));
      p_sum += samples[j].probability;
    }
    if (p_sum < 1e-10)
      p_sum = 1e-10; // hacky divide by zero protection
    for (int j=0; j<num_samples_per_iteration_; ++j)
      samples[j].probability/=p_sum;

    // update solution
    joint_angles.data = Eigen::VectorXd::Zero(chain_.num_joints_);
    covariance = Eigen::MatrixXd::Zero(chain_.num_joints_, chain_.num_joints_);
    for (int j=0; j<num_samples_per_iteration_; ++j)
    {
      joint_angles.data += samples[j].probability * samples[j].joint_angles.data;
      diff = samples[j].joint_angles.data - prev_mean;
      covariance += samples[j].probability * (diff * diff.transpose());
    }

    //for (int k=0; k<chain_.num_joints_; k++)
    //  covariance(k,k) += 1e-2;

    prev_mean = joint_angles.data;

  }
  q_out = joint_angles;
  return true;
}

double StochasticIKSolver::computeCost(const KDL::JntArray& q_in, const KDL::Frame& pose_des, KinematicsInfo& kinematics_info)
{
  double cost = 0.0;
  chain_.fk_solver_->solve(q_in, kinematics_info);
  KDL::Frame pose = kinematics_info.link_frames_.back();
  KDL::Twist error = diff(pose_des, pose);
  cost = position_cost_weight_ * dot(error.vel, error.vel);
  cost += orientation_cost_weight_ * dot(error.rot, error.rot);
  return cost;
}

void StochasticIKSolver::readParams()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, "noise_stddev", noise_stddev_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "noise_decay", noise_decay_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_samples_per_iteration", num_samples_per_iteration_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "max_iterations", max_iterations_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cost_to_probability_h", cost_to_probability_h_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "position_cost_weight", position_cost_weight_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "orientation_cost_weight", orientation_cost_weight_));
}

}
