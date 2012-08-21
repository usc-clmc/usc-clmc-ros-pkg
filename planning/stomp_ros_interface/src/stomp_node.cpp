/*
 * stomp_node.cpp
 *
 *  Created on: Jul 21, 2012
 *      Author: kalakris
 */

#include <ros/ros.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <stomp_ros_interface/stomp_node.h>
#include <stomp_ros_interface/stomp_optimization_task.h>

namespace stomp_ros_interface
{

StompNode::StompNode():
    node_handle_("~")
{
}

StompNode::~StompNode()
{
}

bool StompNode::run()
{
  rviz_trajectory_pub_ = node_handle_.advertise<visualization_msgs::Marker>("trajectories", 20, true);
  collision_models_interface_.reset(new planning_environment::CollisionModelsInterface("robot_description"));

  ros::NodeHandle stomp_task_nh(node_handle_, "task");
  ros::NodeHandle optimizer_task_nh(node_handle_, "optimizer");

  int max_rollouts;
  ROS_VERIFY(optimizer_task_nh.getParam("max_rollouts", max_rollouts));

  XmlRpc::XmlRpcValue planning_groups_xml;
  if (!stomp_task_nh.getParam("planning_groups", planning_groups_xml) || planning_groups_xml.getType()!=XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("planning_groups parameter needs to be an array");
    return false;
  }
  for (int i=0; i<planning_groups_xml.size(); ++i)
  {
    std::string name;
    ROS_VERIFY(usc_utilities::getParam(planning_groups_xml[i], "name", name));

    // for this planning group, create a STOMP task
    boost::shared_ptr<StompOptimizationTask> stomp_task;
    stomp_task.reset(new stomp_ros_interface::StompOptimizationTask(stomp_task_nh, name));
    stomp_task->initialize(8, max_rollouts+1);
    stomp_task->setTrajectoryVizPublisher(rviz_trajectory_pub_);

    // TODO - hardcoded weights for now
    std::vector<double> weights;
    weights.resize(4, 0.0);
    weights[0] = 1.0;
    weights[1] = 20.0;
    weights[2] = 4.0;
    weights[3] = 0.0;
    stomp_task->setControlCostWeight(0.00001);
    stomp_task->setFeatureWeights(weights);

    stomp_tasks_.insert(std::make_pair(name, stomp_task));

    ROS_INFO("Initialized STOMP task for group %s", name.c_str());
  }

  stomp_.reset(new stomp::STOMP());

  if (!collision_models_interface_->loadedModels())
    return false;

  plan_path_service_ = node_handle_.advertiseService("plan_path", &StompNode::plan, this);
  return true;
}

bool StompNode::plan(arm_navigation_msgs::GetMotionPlan::Request& request,
          arm_navigation_msgs::GetMotionPlan::Response& response)
{
  // get the task
  std::map<std::string, boost::shared_ptr<StompOptimizationTask> >::iterator task_it =
      stomp_tasks_.find(request.motion_plan_request.group_name);
  if (task_it == stomp_tasks_.end())
  {
    ROS_ERROR("STOMP: Couldn't find planner group %s", request.motion_plan_request.group_name.c_str());
    response.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GROUP_NAME;
    return true;
  }

  boost::shared_ptr<StompOptimizationTask> task = task_it->second;

  // get and set the planning scene
  task->setPlanningScene(collision_models_interface_->getLastPlanningScene());

  task->setMotionPlanRequest(request.motion_plan_request);
  ros::NodeHandle stomp_optimizer_nh(node_handle_, "optimizer");
  stomp_->initialize(stomp_optimizer_nh, task);

  // TODO: read these params from server
  bool success = stomp_->runUntilValid(200, 30);

  std::vector<Eigen::VectorXd> best_params;
  double best_cost;
  stomp_->getBestNoiselessParameters(best_params, best_cost);
  task->parametersToJointTrajectory(best_params, response.trajectory.joint_trajectory);

  if (!success)
  {
    ROS_ERROR("STOMP: failed to find a collision-free plan");
    response.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::PLANNING_FAILED;
    return true;
  }

  response.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;

  return true;
}

} /* namespace stomp_ros_interface */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stomp");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  stomp_ros_interface::StompNode node;

  if (!node.run())
    return -1;

  ROS_INFO("STOMP: Initialized and waiting for planning requests.");

  ros::waitForShutdown();
  return 0;
}
