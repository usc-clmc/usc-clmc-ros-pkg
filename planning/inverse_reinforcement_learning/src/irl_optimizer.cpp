/*
 * irl_optimizer.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: kalakris
 */

#include <inverse_reinforcement_learning/irl_optimizer.h>
#include <cstdio>
#include <limits>

namespace inverse_reinforcement_learning
{

IRLOptimizer::IRLOptimizer(int num_features):
        num_features_(num_features),
        alpha_(0.0),
        beta_(0.0)
{
  gradient_ = Eigen::VectorXd::Zero(num_features);
  gradient2_ = Eigen::VectorXd::Zero(num_features);
  tmp_gradient_ = Eigen::VectorXd::Zero(num_features);
  weights_ = Eigen::VectorXd::Zero(num_features);
  prev_weights_ = Eigen::VectorXd::Zero(num_features);
  next_weights_ = Eigen::VectorXd::Zero(num_features);
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

void IRLOptimizer::setAlpha(double alpha)
{
  alpha_ = alpha;
}

void IRLOptimizer::setBeta(double beta)
{
  beta_ = beta;
}

void IRLOptimizer::getWeights(Eigen::VectorXd& weights)
{
  weights = weights_;
}

void IRLOptimizer::updateL1Coeff()
{
  int num_active = 0;
  double learning_rate = 1.0;
  double sum_abs_w = 0.0;
  double new_alpha;
  for (int i=0; i<num_features_; ++i)
  {
    if (weights_[i] != 0.0)
      ++num_active;
    sum_abs_w += fabs(weights_[i]);
  }
  if (num_active == 0)
    new_alpha = alpha_ / 2.0;
  else
  {
    if (sum_abs_w < 1e-10)
      sum_abs_w = 1e-10;
    new_alpha = num_active / sum_abs_w;
  }
  alpha_ = learning_rate * (new_alpha)
      + (1.0 - learning_rate) * alpha_;
  printf("New alpha = %f\n", alpha_);
}

void IRLOptimizer::runLBFGS()
{
  // init variables
  lbfgsfloatval_t fx;
  lbfgsfloatval_t* m_x = lbfgs_malloc(num_features_);
  eigenToLbfgs(weights_, m_x);

  // init parameters
  lbfgs_parameter_t params;
  lbfgs_parameter_init(&params);
  params.orthantwise_c = alpha_;
  params.orthantwise_start = 0;
  params.orthantwise_end = num_features_;
  params.max_iterations = 1000;
  if (alpha_ > 0.0)
  {
    params.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
  }

  // call optimization
  int ret = lbfgs(num_features_, m_x, &fx, _evaluate, _progress, this, &params);

  if (ret < 0)
  {
    printf("Error: lbfgs failed with error code %d!\n", ret);
    if (ret == LBFGSERR_MAXIMUMLINESEARCH)
    {
      printf("Line search used max number of evaluations\n");
    }
  }

  lbfgsToEigen(m_x, weights_);

}

void IRLOptimizer::runSBMLRGrid()
{
  // try a bunch of alphas with LBFGS
  alpha_ = pow(2,32);

  double best_alpha = alpha_;
  Eigen::VectorXd best_weights_ = weights_;
  double best_objective = std::numeric_limits<double>::max();
  int best_num_weights = 0;

  for (int i=0; i<64; ++i)
  {
    runLBFGS();
    printf(".");
    fflush(stdout);
    //testGradient();
    int num_active=0;
    double objective = getSBMLRObjective(num_active);
    //printf("alpha = %f\t objective = %f\tnum_active = %d\n", alpha_, objective, num_active);
    if (objective < best_objective)
    {
      best_objective = objective;
      best_alpha = alpha_;
      best_weights_ = weights_;
      best_num_weights = num_active;
    }
    else if (objective > best_objective)
    {
      //printf("objective increasing, we're done!\n");
      break;
    }
    alpha_ = alpha_ * 0.5;
  }
  alpha_ = best_alpha;
  weights_ = best_weights_;
  printf("\nalpha = %f\t objective = %f\tnum_active = %d\n", alpha_, best_objective, best_num_weights);
}

void IRLOptimizer::runSBMLR()
{
  runSBMLRGrid();
  //ternarySearchSBMLR(-32.0, 32.0);
}

void IRLOptimizer::ternarySearchSBMLR(double left, double right)
{
  double abs_tol = 0.1;
  if (right-left < abs_tol)
    return;

  printf("left = %f, right = %f\n", pow(2,left), pow(2,right));

  double left_third = (2.0*left + right)/3.0;
  double right_third = (left + 2.0*right)/3.0;
  alpha_ = pow(2,left_third);
  runLBFGS();
  double f_left_third = getSBMLRObjective();
  printf("alpha = %f\t objective = %f\n", alpha_, f_left_third);
  alpha_ = pow(2,right_third);
  runLBFGS();
  double f_right_third = getSBMLRObjective();
  printf("alpha = %f\t objective = %f\n", alpha_, f_right_third);
  if (f_left_third < f_right_third)
    ternarySearchSBMLR(left, right_third);
  else
    ternarySearchSBMLR(left_third, right);
}

void IRLOptimizer::runSingleIteration()
{

  computeObjectiveAndGradient();
  addL1Gradients();

  prev_weights_ = weights_;

  // get candidate weights
  next_weights_ = weights_ - (gradient_.array()/gradient2_.array()).matrix();

  // if l_1 regularization is used, zero out weights that crossed 0
  if (alpha_ > 0.0)
  {
    for (int i=0; i<num_features_; ++i)
    {
      if (weights_[i] * prev_weights_[i] < 0.0)
        weights_[i] = 0.0;
    }
  }

}

void IRLOptimizer::addL1Gradients()
{
  // add l1 regularization term
  for (int i=0; i<num_features_; ++i)
  {
    if (weights_[i] > 0)
      gradient_[i] += alpha_;
    else if (weights_[i] < 0)
      gradient_[i] -= alpha_;
    else if (gradient_[i] + alpha_ < 0)
      gradient_[i] += alpha_;
    else if (gradient_[i] - alpha_ > 0)
      gradient_[i] -= alpha_;
    else
      gradient_[i] = 0.0;

    objective_ += alpha_ * fabs(weights_[i]);
  }

}

void IRLOptimizer::addL2Gradients()
{
  for (int i=0; i<num_features_; ++i)
  {
    objective_ += beta_ * weights_[i] * weights_[i];
    gradient_[i] += 2.0*beta_ * weights_[i];
  }
}

void IRLOptimizer::computeObjectiveAndGradient()
{
  log_likelihood_ = 0.0;

  // compute gradients
  gradient_ = Eigen::VectorXd::Zero(num_features_);
  gradient2_ = Eigen::VectorXd::Zero(num_features_);

  for (unsigned int i=0; i<data_.size(); ++i)
  {
    double tmp_log_lik = 0.0;
    data_[i]->computeGradient2(weights_, tmp_gradient_, tmp_log_lik, tmp_gradient2_);
    gradient_ += tmp_gradient_;
    gradient2_ += tmp_gradient2_;
    log_likelihood_ += tmp_log_lik;
  }

  objective_ = -log_likelihood_;
}

double IRLOptimizer::getSBMLRObjective(int& num_active)
{
  computeObjectiveAndGradient();
  num_active = 0;
  double sum_abs_w = 0.0;
  for (int i=0; i<num_features_; ++i)
  {
    if (weights_[i] != 0.0)
      ++num_active;
    sum_abs_w += fabs(weights_[i]);
  }
  if (num_active == 0)
    return -log_likelihood_; // this solution is useless for us
//    return std::numeric_limits<double>::max(); // this solution is useless for us
  else
    return -log_likelihood_ + num_active * log(sum_abs_w);
}

double IRLOptimizer::getSBMLRObjective()
{
  int num_active=0;
  return getSBMLRObjective(num_active);
}

void IRLOptimizer::eigenToLbfgs(const Eigen::VectorXd& eigen_vec, lbfgsfloatval_t *lbfgs_vec)
{
  for (int i=0; i<eigen_vec.rows(); ++i)
    lbfgs_vec[i] = eigen_vec[i];
}

void IRLOptimizer::lbfgsToEigen(const lbfgsfloatval_t *lbfgs_vec, Eigen::VectorXd& eigen_vec)
{
  for (int i=0; i<eigen_vec.rows(); ++i)
    eigen_vec[i] = lbfgs_vec[i];
}

lbfgsfloatval_t IRLOptimizer::_evaluate(void* instance,
                                 const lbfgsfloatval_t *x,
                                 lbfgsfloatval_t *g,
                                 const int n,
                                 const lbfgsfloatval_t step)
{
  IRLOptimizer* optimizer = reinterpret_cast<IRLOptimizer*>(instance);

  // set the weights
  //int num_features = optimizer->weights_.rows();
  lbfgsToEigen(x, optimizer->weights_);
  optimizer->computeObjectiveAndGradient();
  optimizer->addL2Gradients();
  //printf("x = %lf, %lf: %lf\n", x[0], x[1], -optimizer->log_likelihood_);
  eigenToLbfgs(optimizer->gradient_, g);

  return optimizer->objective_;
}


int IRLOptimizer::_progress(void *instance,
                     const lbfgsfloatval_t* x,
                     const lbfgsfloatval_t* g,
                     const lbfgsfloatval_t fx,
                     const lbfgsfloatval_t xnorm,
                     const lbfgsfloatval_t gnorm,
                     const lbfgsfloatval_t step,
                     int n,
                     int k,
                     int ls)
{
  //printf("Iteration %d: %f\n", k, fx);
  return 0;
}

void IRLOptimizer::testGradient()
{
  double delta = 1e-6;
  Eigen::VectorXd findiff_gradient = Eigen::VectorXd::Zero(num_features_);
  Eigen::VectorXd analytical_gradient;
  Eigen::VectorXd orig_weights = weights_;

  computeObjectiveAndGradient();
  analytical_gradient = gradient_;

  printf("Gradients: (analytical / measured)");
  for (int i=0; i<num_features_; ++i)
  {
    double neg, pos;
    weights_[i] = orig_weights[i] - delta;
    computeObjectiveAndGradient();
    neg = objective_;
    weights_[i] = orig_weights[i] + delta;
    computeObjectiveAndGradient();
    pos = objective_;
    weights_[i] = orig_weights[i];
    findiff_gradient[i] = (pos-neg)/(2.0*delta);
    printf("%f\t%f\n", analytical_gradient[i], findiff_gradient[i]);
  }

}


} /* namespace inverse_reinforcement_learning */
