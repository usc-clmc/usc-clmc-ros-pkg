/*
 * stomp_optimization_task.h
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#ifndef STOMP_OPTIMIZATION_TASK_H_
#define STOMP_OPTIMIZATION_TASK_H_

#include <stomp/task.h>
#include <stomp_ros_interface/stomp_robot_model.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/MotionPlanRequest.h>
#include <learnable_cost_function/feature_set.h>

namespace stomp_ros_interface
{

class StompOptimizationTask: public stomp::Task
{
public:
  StompOptimizationTask(ros::NodeHandle node_handle);
  virtual ~StompOptimizationTask();

  virtual bool initialize(int num_threads);

  virtual bool execute(std::vector<Eigen::VectorXd>& parameters,
                       Eigen::VectorXd& costs,
                       Eigen::MatrixXd& weighted_feature_values,
                       const int iteration_number,
                       const int rollout_number,
                       int thread_id);

  void computeFeatures(std::vector<Eigen::VectorXd>& parameters,
                       Eigen::MatrixXd& features,
                       int thread_id);

  void computeCosts(const Eigen::MatrixXd& features, Eigen::VectorXd& costs, Eigen::MatrixXd& weighted_feature_values) const;

  void setPlanningScene(const arm_navigation_msgs::PlanningScene& scene);

  void setMotionPlanRequest(const arm_navigation_msgs::MotionPlanRequest& request);

  void setFeatureWeights(std::vector<double> weights);

  void publishTrajectoryMarkers(ros::Publisher& viz_pub);

  struct PerThreadData
  {
    boost::shared_ptr<StompRobotModel> robot_model_;
    const StompRobotModel::StompPlanningGroup* planning_group_;
    std::vector<boost::shared_ptr<StompCostFunctionInput> > cost_function_input_; // one per timestep
    Eigen::MatrixXd features_; // num_time x num_features
    Eigen::MatrixXd weighted_features_; // num_time x num_features
    Eigen::VectorXd costs_;
    void publishMarkers(ros::Publisher& viz_pub, int id, bool noiseless);
  };

  void getRolloutData(PerThreadData& noiseless_rollout, std::vector<PerThreadData>& noisy_rollouts);

  virtual bool getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy);

  virtual bool setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy);

  virtual double getControlCostWeight();

private:
  boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;
  boost::shared_ptr<learnable_cost_function::FeatureSet> feature_set_;
  double control_cost_weight_;
  std::vector<PerThreadData> per_thread_data_;
  std::vector<PerThreadData> noisy_rollout_data_;
  PerThreadData noiseless_rollout_data_;
  boost::shared_ptr<StompCollisionSpace> collision_space_;
  ros::NodeHandle node_handle_;

  Eigen::VectorXd feature_weights_;

  int num_threads_;
  int num_time_steps_;
  int num_dimensions_;
  double movement_duration_;
  std::string reference_frame_;
  std::string planning_group_;
};

} /* namespace stomp_ros_interface */
#endif /* STOMP_OPTIMIZATION_TASK_H_ */
