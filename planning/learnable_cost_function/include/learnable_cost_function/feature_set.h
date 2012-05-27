/*
 * feature_set.h
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#ifndef LEARNABLE_COST_FUNCTION_FEATURE_SET_H_
#define LEARNABLE_COST_FUNCTION_FEATURE_SET_H_

#include <learnable_cost_function/feature.h>

namespace learnable_cost_function
{

struct FeatureInfo
{
  boost::shared_ptr<Feature> feature;
  int num_values;
  std::vector<double> values;
  std::vector<Eigen::VectorXd> gradients;
};

class FeatureSet: public Feature
{
public:
  FeatureSet();
  virtual ~FeatureSet();

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(boost::shared_ptr<Input const> input, std::vector<double>& feature_values,
                                         bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity);
  virtual std::string getName() const;
  virtual boost::shared_ptr<Feature> clone() const;

  void addFeature(boost::shared_ptr<Feature> features);
  void clear();

private:
  std::vector<FeatureInfo> features_;
  int num_values_;

};

} /* namespace stomp_ros_interface */
#endif /* LEARNABLE_COST_FUNCTION_FEATURE_SET_H_ */
