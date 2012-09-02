/*
 * stomp_cost_feature.h
 *
 *  Created on: Aug 31, 2012
 *      Author: kalakris
 */

#ifndef STOMP_COST_FEATURE_H_
#define STOMP_COST_FEATURE_H_

#include <learnable_cost_function/feature.h>
#include <stomp_ros_interface/stomp_robot_model.h>
#include <pluginlib/class_list_macros.h>

namespace stomp_ros_interface
{

class StompCostFeature: public learnable_cost_function::Feature
{
public:
  StompCostFeature(){};
  virtual ~StompCostFeature(){};

  bool initialize(XmlRpc::XmlRpcValue& config) {return false;}  // use function below instead of this one

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group)=0;
};

} /* namespace stomp_ros_interface */
#endif /* STOMP_COST_FEATURE_H_ */
