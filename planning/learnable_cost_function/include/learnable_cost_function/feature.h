/*
 * feature.h
 *
 *  Created on: Jul 27, 2011
 *      Author: kalakris
 */

#ifndef LEARNABLE_COST_FUNCTION_FEATURE_H_
#define LEARNABLE_COST_FUNCTION_FEATURE_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <learnable_cost_function/input.h>
#include <Eigen/Core>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace learnable_cost_function
{

/**
 * A feature evaluates the input and gives out a bunch of feature values. The feature values can then
 * be arbitrarily weighted by "LinearCostFunction" to provide a cost function value.
 */
class Feature
{
public:
  Feature(){};
  virtual ~Feature(){};

  virtual bool initialize(XmlRpc::XmlRpcValue& config) = 0;
  virtual int getNumValues() const = 0;
  virtual void computeValuesAndGradients(boost::shared_ptr<Input const> input, std::vector<double>& feature_values,
                                         bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity) = 0;
  virtual std::string getName() const = 0;
  virtual boost::shared_ptr<Feature> clone() const = 0;

  virtual void getNames(std::vector<std::string>& names) const {};
};

}

#endif /* LEARNABLE_COST_FUNCTION_FEATURE_H_ */
