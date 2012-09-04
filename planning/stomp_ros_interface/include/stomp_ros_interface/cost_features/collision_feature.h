/*
 * collision_feature.h
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#ifndef COLLISION_FEATURE_H_
#define COLLISION_FEATURE_H_

#include <stomp_ros_interface/cost_features/stomp_cost_feature.h>

namespace stomp_ros_interface
{

class CollisionFeature: public StompCostFeature
{
public:
  CollisionFeature();
  virtual ~CollisionFeature();

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group);
  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> input, std::vector<double>& feature_values,
                                         bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity);
  virtual std::string getName() const;
  virtual boost::shared_ptr<learnable_cost_function::Feature> clone() const;

  void getNames(std::vector<std::string>& names) const;

private:
  double num_sigmoids_;
  std::vector<double> sigmoid_centers_;
  std::vector<double> sigmoid_slopes_;
};

} /* namespace stomp_ros_interface */
#endif /* COLLISION_FEATURE_H_ */
