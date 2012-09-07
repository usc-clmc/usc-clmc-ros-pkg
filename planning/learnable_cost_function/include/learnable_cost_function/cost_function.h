/*
 * cost_function.h
 *
 *  Created on: Jul 27, 2011
 *      Author: kalakris
 */

#ifndef COST_FUNCTION_H_
#define COST_FUNCTION_H_

#include <learnable_cost_function/input.h>
#include <learnable_cost_function/feature.h>

namespace learnable_cost_function
{

class CostFunction
{
public:
  CostFunction(){};
  virtual ~CostFunction(){};

  virtual void getValueAndGradient(boost::shared_ptr<Input const> input, double& value,
                           bool compute_gradient, Eigen::VectorXd& gradient, bool& state_validity,
                           Eigen::VectorXd& feature_values) = 0;

  virtual boost::shared_ptr<CostFunction> clone() = 0;

  virtual void setWeights(const Eigen::VectorXd& weights) = 0;

};

}

#endif /* COST_FUNCTION_H_ */
