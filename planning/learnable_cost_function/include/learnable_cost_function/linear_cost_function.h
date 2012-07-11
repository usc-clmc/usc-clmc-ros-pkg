/*
 * linear_cost_function.h
 *
 *  Created on: Jul 27, 2011
 *      Author: kalakris
 */

#ifndef LEARNABLE_COST_FUNCTION_LINEAR_COST_FUNCTION_H_
#define LEARNABLE_COST_FUNCTION_LINEAR_COST_FUNCTION_H_

#include <learnable_cost_function/input.h>
#include <learnable_cost_function/feature.h>
#include <learnable_cost_function/cost_function.h>
#include <learnable_cost_function/feature_set.h>

namespace learnable_cost_function
{

/**
 * Cost function: weighted linear combination of features
 */
class LinearCostFunction: public CostFunction
{
public:
  LinearCostFunction();
  virtual ~LinearCostFunction();

  virtual void getValueAndGradient(boost::shared_ptr<Input const> input, double& value,
                           bool compute_gradient, Eigen::VectorXd& gradient, bool& state_validity,
                           std::vector<double>& weighted_feature_values);

  void clear();
  void addFeaturesAndWeights(std::vector<boost::shared_ptr<Feature> > features,
                             std::vector<double> weights);

  virtual boost::shared_ptr<CostFunction> clone();

  void debugCost(double cost, const std::vector<double>& weighted_feature_values);

private:
  std::vector<FeatureInfo> features_;
  int num_feature_values_;

};

}

#endif /* LEARNABLE_COST_FUNCTION_LINEAR_COST_FUNCTION_H_ */
