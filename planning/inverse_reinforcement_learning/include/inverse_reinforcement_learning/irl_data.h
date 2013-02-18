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
  void addImportanceWeightedSamples(const Eigen::MatrixXd& features, const Eigen::VectorXd& target, const Eigen::VectorXd& log_importance_weights);

  /**
   * Computes the gradient (of the neg log likelihood) and the log likelihood of the data
   */
  void computeGradient(const Eigen::VectorXd& weights, Eigen::VectorXd& gradient, double& log_likelihood);

  /**
   * Same as computeGradient, but also computes second partial derivatives
   */
  void computeGradient2(const Eigen::VectorXd& weights, Eigen::VectorXd& gradient, double& log_likelihood,
                        Eigen::VectorXd& gradient2);

  void saveToFile(const std::string& abs_file_name);
  void loadFromFile(const std::string& abs_file_name);

  int getNumFeatures();
  int getNumSamples();

  /**
   * Gets the average "rank" (between 0 and 1) of the samples with target = 1
   */
  double getRank(const Eigen::VectorXd& weights);

  double getLikelihood(const Eigen::VectorXd& weights);

private:
  Eigen::VectorXd exp_w_phi_;   /**< [num_samples] exp(-w^trans * phi) */
  Eigen::VectorXd y_;           /**< likelihood */

  int num_samples_;             /**< number of samples */
  int num_features_;            /**< number of features */
  Eigen::VectorXd target_;      /**< [num_samples] target variables: 1 = good, 0 = bad */
  Eigen::MatrixXd features_;    /**< [num_samples x num_features] */
  Eigen::VectorXd log_importance_weights_;     /**< [num_samples] importance sampling weight for each data point */

  void computeLikelihoods(const Eigen::VectorXd& weights);
};

} /* namespace inverse_reinforcement_learning */
#endif /* IRL_DATA_H_ */
