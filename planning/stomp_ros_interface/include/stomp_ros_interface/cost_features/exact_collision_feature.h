/*
 * exact_collision_feature.h
 *
 *  Created on: Jul 22, 2012
 *      Author: kalakris
 */

#ifndef EXACT_COLLISION_FEATURE_H_
#define EXACT_COLLISION_FEATURE_H_

#include <learnable_cost_function/feature.h>
#include <planning_environment/models/collision_models.h>

namespace stomp_ros_interface
{

class ExactCollisionFeature: public learnable_cost_function::Feature
{
public:
  ExactCollisionFeature();
  virtual ~ExactCollisionFeature();

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> input, std::vector<double>& feature_values,
                                         bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity);
  virtual std::string getName() const;
  virtual boost::shared_ptr<learnable_cost_function::Feature> clone() const;

};

} /* namespace stomp_ros_interface */
#endif /* EXACT_COLLISION_FEATURE_H_ */
