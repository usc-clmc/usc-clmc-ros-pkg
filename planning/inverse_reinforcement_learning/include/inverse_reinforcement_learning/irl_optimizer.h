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
   * Run L-BFGS with L-1 regularization
   */
  void runLBFGS();

  /**
   * Run Sparse Bayesian Multinomial Logistic Regression
   */
  void runSBMLR();
  void runSBMLRGrid();

  void getWeights(Eigen::VectorXd& weights);

private:
  int num_features_;
  std::vector<boost::shared_ptr<IRLData> > data_;
  Eigen::VectorXd weights_;
  Eigen::VectorXd prev_weights_;
  double alpha_;                                        /**< l1 regularization term multiplier */
  double learning_rate_;
  double log_likelihood_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd gradient2_;

  //tmp variables
  Eigen::VectorXd tmp_gradient_;
  Eigen::VectorXd tmp_gradient2_;
  Eigen::VectorXd next_weights_;


  void computeObjectiveAndGradient();
  void addL1Gradients();
  void updateL1Coeff();
  void runSingleIteration();

  void ternarySearchSBMLR(double left, double right);

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
