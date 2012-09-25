/*
 * irl_optimizer.h
 *
 *  Created on: Aug 28, 2012
 *      Author: kalakris
 */

#ifndef IRL_OPTIMIZER_H_
#define IRL_OPTIMIZER_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <inverse_reinforcement_learning/irl_data.h>
#include <lbfgs/lbfgs.h>

namespace inverse_reinforcement_learning
{

class IRLOptimizer
{
public:
  IRLOptimizer(int num_features);
  virtual ~IRLOptimizer();

  void setData(const std::vector<boost::shared_ptr<IRLData> >& data_);
  void setWeights(const Eigen::VectorXd& weights);

  /**
   * Set l1 regularization coefficient
   */
  void setAlpha(double alpha);

  /**
   * Set l2 regularization coefficient
   */
  void setBeta(double beta);

  /**
   * Run L-BFGS with L-1 and L-2 regularization
   */
  void runLBFGS();

  /**
   * Optimize the objective function, choosing the best alpha by cross validation
   */
  void optimizeWithCV(int num_cv=2);

  /**
   * Run Sparse Bayesian Multinomial Logistic Regression
   */
  void runSBMLR();
  void runSBMLRGrid();
  void runSBMLRTernarySearch(double min_alpha, double max_alpha);

  void getWeights(Eigen::VectorXd& weights);

  void testGradient();

  void computeAverageRank();

  void normalizeData();
  void unnormalizeData();

  static double compareWeights(Eigen::VectorXd weights1,
                             Eigen::VectorXd weights2);

  void useAllData();

private:
  int num_features_;
  std::vector<boost::shared_ptr<IRLData> > data_;
  std::vector<int> active_data_;                      /**< these are the data elements currently used in evaluating objective functions */
  Eigen::VectorXd weights_;
  Eigen::VectorXd prev_weights_;
  Eigen::VectorXd means_;
  Eigen::VectorXd variances_;
  double alpha_;                                        /**< l1 regularization term multiplier */
  double beta_;                                         /**< l2 regularization term multiplier */
  double learning_rate_;
  double log_likelihood_;
  double objective_;
  double l1_objective_;
  double l2_objective_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd gradient2_;
  int num_weights_active_;

  //tmp variables
  Eigen::VectorXd tmp_gradient_;
  Eigen::VectorXd tmp_gradient2_;
  Eigen::VectorXd next_weights_;


  void computeObjectiveAndGradient();
  void addL1Gradients();
  void addL2Gradients();
  void updateL1Coeff();
  void runSingleIteration();

  void ternarySearchSBMLR(double left, double right, double f_left, double f_right,
                          const Eigen::VectorXd& left_weights, const Eigen::VectorXd& right_weights);

  double getSBMLRObjective(int& num_weights_active);
  double getSBMLRObjective();

  // supporting functions for LBFGS optimization

  static void eigenToLbfgs(const Eigen::VectorXd& eigen_vec, lbfgsfloatval_t *lbfgs_vec);
  static void lbfgsToEigen(const lbfgsfloatval_t *lbfgs_vec, Eigen::VectorXd& eigen_vec);

  static lbfgsfloatval_t _evaluate(void* instance,
                                   const lbfgsfloatval_t *x,
                                   lbfgsfloatval_t *g,
                                   const int n,
                                   const lbfgsfloatval_t step);
  static int _progress(void *instance,
                       const lbfgsfloatval_t* x,
                       const lbfgsfloatval_t* g,
                       const lbfgsfloatval_t fx,
                       const lbfgsfloatval_t xnorm,
                       const lbfgsfloatval_t gnorm,
                       const lbfgsfloatval_t step,
                       int n,
                       int k,
                       int ls);

};

} /* namespace inverse_reinforcement_learning */
#endif /* IRL_OPTIMIZER_H_ */
