/*
 * irl_data.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: kalakris
 */

#include <inverse_reinforcement_learning/irl_data.h>
#include <ros/assert.h>

namespace inverse_reinforcement_learning
{

IRLData::IRLData(int num_features):
    num_samples_(0),
    num_features_(num_features)
{
}

IRLData::~IRLData()
{

}

void IRLData::addSamples(const Eigen::MatrixXd& features, const Eigen::VectorXd& target)
{
  ROS_ASSERT(features.rows() == target.rows());
  ROS_ASSERT(features.cols() == num_features_);

  int new_num_samples = features.rows();
  int total_num_samples = new_num_samples + num_samples_;
  Eigen::MatrixXd new_features = Eigen::MatrixXd(total_num_samples, num_features_);
  Eigen::VectorXd new_target = Eigen::VectorXd(total_num_samples);

  if (num_samples_ > 0)
    new_features.topRows(num_samples_) = features_;
  new_features.bottomRows(new_num_samples) = features;
  features_ = new_features;

  if (num_samples_ > 0)
    new_target.head(num_samples_) = target_;
  new_target.tail(new_num_samples) = target;
  target_ = target;

  num_samples_ = total_num_samples;
}

void IRLData::computeGradient(const Eigen::VectorXd& weights, Eigen::VectorXd& gradient)
{
  ROS_ASSERT(weights.rows() == num_features_);
  exp_w_phi_ = (-features_ * weights).array().exp().matrix();
  double sum_exp_w_phi = exp_w_phi_.sum();
  if (sum_exp_w_phi < 1e-6)
    sum_exp_w_phi = 1e-6;
  y_ = exp_w_phi_ / sum_exp_w_phi;
  gradient = ((target_ - y_).transpose() * features_).transpose();
}

} /* namespace inverse_reinforcement_learning */
