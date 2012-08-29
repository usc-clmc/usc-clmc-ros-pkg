/*
 * irl_data.h
 *
 *  Created on: Aug 28, 2012
 *      Author: kalakris
 */

#ifndef IRL_DATA_H_
#define IRL_DATA_H_

#include <Eigen/Core>

namespace inverse_reinforcement_learning
{

/**
 * Contains data for an IRL problem
 */
class IRLData
{
public:
  IRLData(int num_features);
  virtual ~IRLData();

  void addSamples(const Eigen::MatrixXd& features, const Eigen::VectorXd& target);

  void computeGradient(const Eigen::VectorXd& weights, Eigen::VectorXd& gradient);

private:
  Eigen::VectorXd exp_w_phi_;   /**< [num_samples] exp(-w^trans * phi) */
  Eigen::VectorXd y_;           /**< likelihood */

  int num_samples_;             /**< number of samples */
  int num_features_;            /**< number of features */
  Eigen::VectorXd target_;      /**< [num_samples] target variables: 1 = good, 0 = bad */
  Eigen::MatrixXd features_;    /**< [num_samples x num_features] */

};

} /* namespace inverse_reinforcement_learning */
#endif /* IRL_DATA_H_ */
