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
//#include <stomp_ros_interface/stomp_cost_function_input.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/MotionPlanRequest.h>
#include <learnable_cost_function/feature_set.h>
#include <planning_environment/models/collision_models_interface.h>
#include <stomp_ros_interface/stomp_collision_space.h>

namespace stomp_ros_interface
{

class StompCostFunctionInput;

class StompOptimizationTask: public stomp::Task
{
  friend class StompCostFunctionInput;

public:
  StompOptimizationTask(ros::NodeHandle node_handle,
                        const std::string& planning_group);
  virtual ~StompOptimizationTask();

  virtual bool initialize(int num_threads, int num_rollouts);

  void setFeatures(std::vector<boost::shared_ptr<learnable_cost_function::Feature> >& features);

  virtual bool execute(std::vector<Eigen::VectorXd>& parameters,
                       std::vector<Eigen::VectorXd>& projected_parameters,
                       Eigen::VectorXd& costs,
                       Eigen::MatrixXd& weighted_feature_values,
                       const int iteration_number,
                       const int rollout_number,
                       int thread_id,
                       bool compute_gradients,
                       std::vector<Eigen::VectorXd>& gradients,
                       bool& validity);

  virtual bool filter(std::vector<Eigen::VectorXd>& parameters, int thread_id);

  void computeFeatures(std::vector<Eigen::VectorXd>& parameters,
                       Eigen::MatrixXd& features,
                       int rollout_id,
                       bool& validity);

  void computeCosts(const Eigen::MatrixXd& features, Eigen::VectorXd& costs, Eigen::MatrixXd& weighted_feature_values) const;

  void setPlanningScene(const arm_navigation_msgs::PlanningScene& scene);

  void setMotionPlanRequest(const arm_navigation_msgs::MotionPlanRequest& request);

  void setInitialTrajectory(const std::vector<sensor_msgs::JointState>& joint_states);

  void setFeatureWeights(std::vector<double> weights);
  void setFeatureWeightsFromFile(const std::string& abs_file_name);
  void setFeatureScaling(std::vector<double> means, std::vector<double> variances);
  void setFeatureScalingFromFile(const std::string& abs_means_file,
                                 const std::string& abs_variance_file);

  void publishTrajectoryMarkers(ros::Publisher& viz_pub);

  void publishCollisionModelMarkers(int rollout_number);

  void parametersToJointTrajectory(const std::vector<Eigen::VectorXd>& parameters, trajectory_msgs::JointTrajectory& trajectory);

  struct PerRolloutData
  {
    boost::shared_ptr<planning_environment::CollisionModels> collision_models_;
    planning_models::KinematicState* kinematic_state_;
    planning_models::KinematicState::JointStateGroup* joint_state_group_;

    //const StompRobotModel::StompPlanningGroup* planning_group_;
    const StompOptimizationTask* task_;

    std::vector<boost::shared_ptr<StompCostFunctionInput> > cost_function_input_; // one per timestep

    Eigen::MatrixXd features_; // num_time x num_features
    Eigen::MatrixXd weighted_features_; // num_time x num_features
    Eigen::VectorXd costs_;

    // temp data structures for differentiation
    std::vector<Eigen::VectorXd> tmp_joint_angles_;     // one per dimension
    std::vector<Eigen::VectorXd> tmp_joint_angles_vel_; // one per dimension
    std::vector<Eigen::VectorXd> tmp_joint_angles_acc_; // one per dimension
    std::vector<std::vector<Eigen::VectorXd> > tmp_collision_point_pos_; // [collision_point_index][x/y/z]
    std::vector<std::vector<Eigen::VectorXd> > tmp_collision_point_vel_; // [collision_point_index][x/y/z]
    std::vector<std::vector<Eigen::VectorXd> > tmp_collision_point_acc_; // [collision_point_index][x/y/z]

    boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver_;
    void differentiate(double dt);
    void publishMarkers(ros::Publisher& viz_pub, int id, bool noiseless, const std::string& reference_frame);
  };

  void getRolloutData(PerRolloutData& noiseless_rollout, std::vector<PerRolloutData>& noisy_rollouts);

  virtual bool getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy);

  virtual bool setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy);

  virtual double getControlCostWeight();
  void setControlCostWeight(double w);

  virtual void onEveryIteration();
  void setTrajectoryVizPublisher(ros::Publisher& viz_trajectory_pub);

  const StompRobotModel::StompPlanningGroup* getPlanningGroup();


private:
  boost::shared_ptr<StompRobotModel> robot_model_;
  const StompRobotModel::StompPlanningGroup* planning_group_;

  boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;
  boost::shared_ptr<learnable_cost_function::FeatureSet> feature_set_;
  double control_cost_weight_;
  //std::vector<PerThreadData> per_thread_data_;
  std::vector<PerRolloutData> per_rollout_data_;
  boost::shared_ptr<StompCollisionSpace> collision_space_;
  ros::NodeHandle node_handle_;

  Eigen::VectorXd feature_weights_;
  Eigen::VectorXd feature_means_;
  Eigen::VectorXd feature_variances_;

  int num_threads_;
  int num_rollouts_;
  int num_time_steps_;
  int num_dimensions_;
  double movement_duration_;
  double dt_;
  std::string reference_frame_;
  std::string planning_group_name_;

  std::vector<double> start_joints_;
  std::vector<double> goal_joints_;

  ros::Publisher viz_pub_;
  ros::Publisher viz_trajectory_pub_;
  bool publish_trajectory_markers_;
  int max_rollout_markers_published_;
  int last_executed_rollout_;

  // variables to handle splitting features based on time
  int num_feature_basis_functions_;
  std::vector<double> feature_basis_centers_;
  std::vector<double> feature_basis_stddev_;
  int num_features_;            // original number of features
  int num_split_features_;      // number of features after "time-split"
  Eigen::MatrixXd feature_basis_functions_; // num_time x num_basis_functions

  static bool loadDoubleArrayFromFile(const std::string& abs_file_name, std::vector<double>& array);

};

} /* namespace stomp_ros_interface */
#endif /* STOMP_OPTIMIZATION_TASK_H_ */
