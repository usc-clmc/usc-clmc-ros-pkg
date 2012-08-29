/*
 * irl_optimizer.h
 *
 *  Created on: Aug 28, 2012
 *      Author: kalakris
 */

#ifndef IRL_OPTIMIZER_H_
#define IRL_OPTIMIZER_H_

#include <inverse_reinforcement_learning/irl_data.h>

namespace inverse_reinforcement_learning
{

class IRLOptimizer
{
public:
  IRLOptimizer();
  virtual ~IRLOptimizer();

  void setData(const std::vector<boost::shared_ptr<IRLData> >& data_);
  void setWeights(const Eigen::VectorXd& weights);

  void runSingleIteration();

private:
  std::vector<boost::shared_ptr<IRLData> > data_;
  Eigen::VectorXd weights_;
  Eigen::VectorXd prev_weights_;
  double alpha_;                                        /**< l1 regularization term multiplier */
  double learning_rate_;

  // tmp variables
  Eigen::VectorXd gradient_;
  Eigen::VectorXd gradient_sum_;
};

} /* namespace inverse_reinforcement_learning */
#endif /* IRL_OPTIMIZER_H_ */
