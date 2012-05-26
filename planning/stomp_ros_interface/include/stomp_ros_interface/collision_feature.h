/*
 * collision_feature.h
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#ifndef COLLISION_FEATURE_H_
#define COLLISION_FEATURE_H_

#include <learnable_cost_function/feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

namespace stomp_ros_interface
{

class CollisionFeature: public learnable_cost_function::Feature
{
public:
  CollisionFeature();
  virtual ~CollisionFeature();

  virtual bool initialize(XmlRpc::XmlRpcValue& config);
  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> input, std::vector<double>& feature_values,
                                         bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity);
  virtual std::string getName() const;
  virtual boost::shared_ptr<learnable_cost_function::Feature> clone() const;

};

} /* namespace stomp_ros_interface */
#endif /* COLLISION_FEATURE_H_ */
