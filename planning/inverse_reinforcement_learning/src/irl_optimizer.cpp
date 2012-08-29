/*
 * irl_optimizer.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: kalakris
 */

#include "irl_optimizer.h"

namespace inverse_reinforcement_learning
{

IRLOptimizer::IRLOptimizer()
{
  alpha_ = 0.0;
}

IRLOptimizer::~IRLOptimizer()
{
}

void IRLOptimizer::setData(const std::vector<boost::shared_ptr<IRLData> >& data)
{
  data_ = data;
}

void IRLOptimizer::setWeights(const Eigen::VectorXd& weights)
{
  weights_ = weights;
}

void IRLOptimizer::runSingleIteration()
{
  int num_features = weights_.rows();

  // compute gradients
  gradient_sum_ = Eigen::VectorXd::Zero(num_features);
  for (unsigned int i=0; i<data_.size(); ++i)
  {
    data_[i]->computeGradient(weights_, gradient_);
    gradient_sum_ += gradient_;
  }

  // add l1 regularization term
  for (int i=0; i<num_features; ++i)
  {
    if (weights_[i] > 0)
      gradient_sum_[i] += alpha_;
    else if (weights_[i] < 0)
      gradient_sum_[i] -= alpha_;
    else if (gradient_sum_[i] + alpha_ < 0)
      gradient_sum_[i] += alpha_;
    else if (gradient_sum_[i] - alpha_ > 0)
      gradient_sum_[i] -= alpha_;
    else
      gradient_sum_[i] = 0.0;
  }

  prev_weights_ = weights_;

  // update the weights
  weights_ = weights_ - learning_rate_ * gradient_sum_;

  // if l_1 regularization is used, zero out weights that crossed 0
  if (alpha_ > 0.0)
  {
    for (int i=0; i<num_features; ++i)
    {
      if (weights_[i] * prev_weights_[i] < 0.0)
        weights_[i] = 0.0;
    }
  }

}

} /* namespace inverse_reinforcement_learning */
