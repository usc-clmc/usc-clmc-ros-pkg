/*
 * stomp_optimization_task.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/stomp_optimization_task.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/file_io.h>
#include <stomp/stomp_utils.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>
#include <iostream>
#include <fstream>

namespace stomp_ros_interface
{

StompOptimizationTask::StompOptimizationTask(ros::NodeHandle node_handle,
                                             const std::string& planning_group):
    node_handle_(node_handle),
    planning_group_name_(planning_group),
    feature_loader_("stomp_ros_interface", "stomp_ros_interface::StompCostFeature")
{
  viz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("robot_model_array", 10, true);
  max_rollout_markers_published_ = 0;
}

StompOptimizationTask::~StompOptimizationTask()
{
}

bool StompOptimizationTask::initialize(int num_threads, int num_rollouts)
{
  num_threads_ = num_threads;
  num_rollouts_ = num_rollouts;
  usc_utilities::read(node_handle_, "reference_frame", reference_frame_);
  usc_utilities::read(node_handle_, "num_feature_basis_functions", num_feature_basis_functions_);

  robot_model_.reset(new StompRobotModel(node_handle_));
  robot_model_->init(reference_frame_);
  planning_group_ = robot_model_->getPlanningGroup(planning_group_name_);
  double max_radius_clearance = robot_model_->getMaxRadiusClearance();
  num_dimensions_ = planning_group_->num_joints_;

  // initialize per-thread-data
  per_rollout_data_.resize(num_rollouts_+1);
  for (int i=0; i<num_rollouts_+1; ++i)
  {
    per_rollout_data_[i].collision_models_.reset(new planning_environment::CollisionModels("/robot_description"));
    per_rollout_data_[i].collision_models_->disableCollisionsForNonUpdatedLinks(planning_group_name_, true);
  }

  //noisy_rollout_data_.resize(num_rollouts_);

  collision_space_.reset(new StompCollisionSpace(node_handle_));
  collision_space_->init(max_radius_clearance, reference_frame_);

  control_cost_weight_ = 0.0;

  last_executed_rollout_ = -1;

  return true;
}

void StompOptimizationTask::setFeaturesFromXml(const XmlRpc::XmlRpcValue& features_xml)
{
  ROS_ASSERT (features_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::vector<boost::shared_ptr<learnable_cost_function::Feature> > features;

  for (int i=0; i<features_xml.size(); ++i)
  {
    XmlRpc::XmlRpcValue feature_xml = features_xml[i];

    ROS_ASSERT(feature_xml.hasMember("class") &&
               feature_xml["class"].getType() == XmlRpc::XmlRpcValue::TypeString);

    std::string class_name = feature_xml["class"];

    boost::shared_ptr<StompCostFeature> feature;
    try
    {
      feature = feature_loader_.createInstance(class_name);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("Couldn't load feature named %s", class_name.c_str());
      ROS_ERROR("Error: %s", ex.what());
      ROS_BREAK();
    }

    feature->initialize(feature_xml, planning_group_);
    features.push_back(feature);
  }

  setFeatures(features);

}

void StompOptimizationTask::setFeatures(std::vector<boost::shared_ptr<learnable_cost_function::Feature> >& features)
{
  // create the feature set
  feature_set_.reset(new learnable_cost_function::FeatureSet());

  for (unsigned int i=0; i<features.size(); ++i)
  {
    feature_set_->addFeature(features[i]);
  }

//  // create features and add them
//  feature_set_->addFeature(boost::shared_ptr<learnable_cost_function::Feature>(new CollisionFeature()));
//  feature_set_->addFeature(boost::shared_ptr<learnable_cost_function::Feature>(
//      new JointVelAccFeature(per_thread_data_[0].planning_group_->num_joints_)));
//  feature_set_->addFeature(boost::shared_ptr<learnable_cost_function::Feature>(new CartesianVelAccFeature()));
//  feature_set_->addFeature(boost::shared_ptr<learnable_cost_function::Feature>(new CartesianOrientationFeature()));

  // init feature splits
  num_features_ = feature_set_->getNumValues();
  ROS_INFO("STOMP: Using %d features classes with %d values", features.size(), num_features_);
  num_split_features_ = num_features_ * num_feature_basis_functions_;

  feature_basis_centers_.resize(num_feature_basis_functions_);
  feature_basis_stddev_.resize(num_feature_basis_functions_);
  double separation = 100.0;
  if (num_feature_basis_functions_ > 1)
    separation = (1.0 / (num_feature_basis_functions_-1));
  for (int i=0; i<num_feature_basis_functions_; ++i)
  {
    feature_basis_centers_[i] = i * separation;
    feature_basis_stddev_[i] = 0.5 * separation;
  }

  // TODO remove initial value hardcoding here
  feature_weights_ = Eigen::VectorXd::Ones(num_split_features_);
  feature_means_ = Eigen::VectorXd::Zero(num_split_features_);
  feature_variances_ = Eigen::VectorXd::Ones(num_split_features_);

  std::vector<std::string> feature_names;
  feature_set_->getNames(feature_names);
  ROS_INFO("Features loaded:");
  for (int i=0; i<num_features_; ++i)
  {
    ROS_INFO("%2d) %s", i, feature_names[i].c_str());
  }
}

bool StompOptimizationTask::filter(std::vector<Eigen::VectorXd>& parameters, int thread_id)
{
  // clip at joint limits
  bool filtered = false;
  for (int d=0; d<num_dimensions_; ++d)
  {
    const StompRobotModel::StompJoint& joint = planning_group_->stomp_joints_[d];
    if (joint.has_joint_limits_)
    {
      for (int t=0; t<num_time_steps_; ++t)
      {
        if (parameters[d](t) > joint.joint_limit_max_)
        {
          parameters[d](t) = joint.joint_limit_max_;
          filtered = true;
        }
        else if (parameters[d](t) < joint.joint_limit_min_)
        {
          parameters[d](t) = joint.joint_limit_min_;
          filtered = true;
        }
      }
    }
  }
  return filtered;
}

bool StompOptimizationTask::execute(std::vector<Eigen::VectorXd>& parameters,
                     std::vector<Eigen::VectorXd>& projected_parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id,
                     bool compute_gradients,
                     std::vector<Eigen::VectorXd>& gradients,
                     bool& validity)
{
  int rollout_id = num_rollouts_;
  if (rollout_number >= 0)
  {
    rollout_id = rollout_number;
    last_executed_rollout_ = rollout_number;
  }
  computeFeatures(parameters, per_rollout_data_[rollout_id].features_, rollout_id, validity);
  computeCosts(per_rollout_data_[rollout_id].features_, costs, weighted_feature_values);

//  // copying it to per_rollout_data
//  PerThreadData* rdata = &noiseless_rollout_data_;
//  if (rollout_number >= 0)
//  {
//    if ((int)noisy_rollout_data_.size() <= rollout_number)
//    {
//      ROS_WARN("Not enough storage space for rollouts!");
//    }
//    rdata = &(noisy_rollout_data_[rollout_number]);
//    last_executed_rollout_ = rollout_number;
//  }
//  *rdata = per_thread_data_[thread_id];
//  // duplicate the cost function inputs
//  for (unsigned int i=0; i<rdata->cost_function_input_.size(); ++i)
//  {
//    boost::shared_ptr<StompCostFunctionInput> input(new StompCostFunctionInput(
//        collision_space_, rdata->robot_model_, rdata->planning_group_));
//    *input = *rdata->cost_function_input_[i];
//    rdata->cost_function_input_[i] = input;
//  }
  return true;
}

void StompOptimizationTask::PerRolloutData::differentiate(double dt)
{
  int num_time_steps = cost_function_input_.size();
  int num_joint_angles = task_->planning_group_->num_joints_;
  int num_collision_points = task_->planning_group_->collision_points_.size();

  // copy to temp structures
  for (int t=0; t<num_time_steps; ++t)
  {
    for (int c=0; c<num_collision_points; ++c)
    {
      for (int d=0; d<3; ++d)
      {
        tmp_collision_point_pos_[c][d](t) = cost_function_input_[t]->collision_point_pos_[c][d];
      }
    }
    for (int j=0; j<num_joint_angles; ++j)
    {
      tmp_joint_angles_[j](t) = cost_function_input_[t]->joint_angles_(j);
    }
  }

  // do differentiation
  for (int j=0; j<num_joint_angles; ++j)
  {
    stomp::differentiate(tmp_joint_angles_[j], stomp::STOMP_VELOCITY,
                         tmp_joint_angles_vel_[j], dt);
    stomp::differentiate(tmp_joint_angles_[j], stomp::STOMP_ACCELERATION,
                         tmp_joint_angles_acc_[j], dt);
  }
  for (int c=0; c<num_collision_points; ++c)
  {
    for (int d=0; d<3; ++d)
    {
      stomp::differentiate(tmp_collision_point_pos_[c][d], stomp::STOMP_VELOCITY,
                           tmp_collision_point_vel_[c][d], dt);
      stomp::differentiate(tmp_collision_point_pos_[c][d], stomp::STOMP_ACCELERATION,
                           tmp_collision_point_acc_[c][d], dt);
    }
  }

  // copy the differentiated data back
  for (int t=0; t<num_time_steps; ++t)
  {
    for (int c=0; c<num_collision_points; ++c)
    {
      for (int d=0; d<3; ++d)
      {
        cost_function_input_[t]->collision_point_vel_[c][d] = tmp_collision_point_vel_[c][d](t);
        cost_function_input_[t]->collision_point_acc_[c][d] = tmp_collision_point_acc_[c][d](t);
      }
    }
    for (int j=0; j<num_joint_angles; ++j)
    {
      cost_function_input_[t]->joint_angles_vel_(j) = tmp_joint_angles_vel_[j](t);
      cost_function_input_[t]->joint_angles_acc_(j) = tmp_joint_angles_acc_[j](t);
    }
  }
}

boost::shared_ptr<StompOptimizationTask::PerRolloutData> StompOptimizationTask::PerRolloutData::clone()
{
  boost::shared_ptr<StompOptimizationTask::PerRolloutData> ret(new StompOptimizationTask::PerRolloutData());

  *ret = *this;
  // duplicate cost function inputs
  for (unsigned int i=0; i<cost_function_input_.size(); ++i)
  {
    ret->cost_function_input_[i] = this->cost_function_input_[i]->clone();
  }

  return ret;
}

void StompOptimizationTask::computeFeatures(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::MatrixXd& features,
                     int rollout_id,
                     bool& validity)
{
  PerRolloutData *data = &per_rollout_data_[rollout_id];

  // prepare the cost function input
  std::vector<double> temp_features(feature_set_->getNumValues());
  std::vector<Eigen::VectorXd> temp_gradients(feature_set_->getNumValues());
  std::vector<double> joint_angles(num_dimensions_);

  // do all forward kinematics
  validity = true;
  bool state_validity;
  for (int t=0; t<num_time_steps_; ++t)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      data->cost_function_input_[t]->joint_angles_(d) = parameters[d](t);
      joint_angles[d] = parameters[d](t);
    }
    data->cost_function_input_[t]->doFK(data->fk_solver_);
    data->cost_function_input_[t]->per_rollout_data_ = data;
    data->cost_function_input_[t]->time_ = t*dt_;
    data->cost_function_input_[t]->time_index_ = t;
  }

  data->differentiate(dt_);

  // set end-effector frames
  for (int t=0; t<num_time_steps_; ++t)
  {
    data->cost_function_input_[t]->endeffector_frame_ =
        data->cost_function_input_[t]->segment_frames_[planning_group_->end_effector_segment_index_];
  }
  // get end-effector velocities
  for (int t=0; t<num_time_steps_; ++t)
  {
    int t2 = t+1;
    if (t2>=num_time_steps_) t2 = t;
    data->cost_function_input_[t]->endeffector_vel_ = KDL::diff(
        data->cost_function_input_[t]->endeffector_frame_,
        data->cost_function_input_[t2]->endeffector_frame_,
        dt_);
  }
  // get end-effector accelerations
  for (int t=0; t<num_time_steps_; ++t)
  {
    int t2 = t+1;
    if (t2>=num_time_steps_) t2 = t;
    data->cost_function_input_[t]->endeffector_acc_ = KDL::diff(
        data->cost_function_input_[t]->endeffector_vel_,
        data->cost_function_input_[t2]->endeffector_vel_,
        dt_);
  }

  // actually compute features
  bool validities[num_time_steps_];
  for (int t=0; t<num_time_steps_; ++t)
  {
    feature_set_->computeValuesAndGradients(data->cost_function_input_[t],
                                            temp_features, false, temp_gradients, state_validity);
    validities[t] = state_validity;
    for (unsigned int f=0; f<temp_features.size(); ++f)
    {
      features.block(t, f*num_feature_basis_functions_, 1, num_feature_basis_functions_) =
          temp_features[f] * feature_basis_functions_.row(t);
//      features(t,f) = temp_features[f];
    }

  }

  // print validities
  if (rollout_id == num_rollouts_)
  {
    std::stringstream ss;
    ss << "[";
    for (int t=0; t<num_time_steps_; ++t)
    {
      if (validities[t])
        ss << "_";
      else
        ss << "X";
    }
    ss << "]";
    ROS_INFO("%s", ss.str().c_str());
  }

  // compute validity
  for (int t=0; t<num_time_steps_; ++t)
  {
    if (t <= 0.1*num_time_steps_)
    {
      if (validities[0] && !validities[t])
        validity = false;
    }
    else if (t >= 0.9*num_time_steps_)
    {
      if (validities[num_time_steps_-1] && !validities[t])
        validity = false;
    }
    else
    {
      if (!validities[t])
        validity = false;
    }
  }

}

void StompOptimizationTask::computeCosts(const Eigen::MatrixXd& features, Eigen::VectorXd& costs, Eigen::MatrixXd& weighted_feature_values) const
{
  weighted_feature_values = features; // just to initialize the size
  for (int t=0; t<num_time_steps_; ++t)
  {
    weighted_feature_values.row(t) = (((features.row(t) - feature_means_.transpose()).array() / feature_variances_.array().transpose()) * feature_weights_.array().transpose()).matrix();
    //weighted_feature_values.row(t) = (features.row(t).array() * feature_weights_.array().transpose()).matrix();
  }
  costs = weighted_feature_values.rowwise().sum();
}


bool StompOptimizationTask::getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy)
{
  policy = policy_;
  return true;
}

bool StompOptimizationTask::setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy)
{
  policy_ = policy;
  return true;
}

double StompOptimizationTask::getControlCostWeight()
{
  return control_cost_weight_;
}

void StompOptimizationTask::setControlCostWeight(double w)
{
  control_cost_weight_ = w;
}

void StompOptimizationTask::setPlanningScene(const arm_navigation_msgs::PlanningScene& scene)
{
  collision_space_->setPlanningScene(scene);
  for (unsigned int i=0; i<per_rollout_data_.size(); ++i)
  {
    if (per_rollout_data_[i].collision_models_->isPlanningSceneSet())
      per_rollout_data_[i].collision_models_->revertPlanningScene(per_rollout_data_[i].kinematic_state_);
    planning_models::KinematicState* kin_state = per_rollout_data_[i].collision_models_->setPlanningScene(scene);
    per_rollout_data_[i].kinematic_state_ = kin_state;
    per_rollout_data_[i].joint_state_group_ = kin_state->getJointStateGroup(planning_group_name_);
  }
}

void StompOptimizationTask::setMotionPlanRequest(const arm_navigation_msgs::MotionPlanRequest& request)
{
  // TODO this is obviously not complete

  // get the start and goal positions from the message
  std::vector<double> start(num_dimensions_, 0.0);
  std::vector<double> goal(num_dimensions_, 0.0);

  const StompRobotModel::StompPlanningGroup* group = planning_group_;
  start = group->getJointArrayFromJointState(request.start_state.joint_state);
  goal = group->getJointArrayFromGoalConstraints(request.goal_constraints);

  start_joints_ = start;
  goal_joints_ = goal;
//  ROS_INFO("start, goal:");
//  for (int i=0; i<num_dimensions_; ++i)
//  {
//    ROS_INFO("%f -> %f", start[i], goal[i]);
//  }

  movement_duration_ = request.expected_path_duration.toSec();
  num_time_steps_ = lrint(movement_duration_/request.expected_path_dt.toSec()) + 1;
  if (num_time_steps_ < 10)
    num_time_steps_ = 10; // may be a hack, but we need some room to plan in :)
  if (num_time_steps_ > 500)
    num_time_steps_ = 500; // may start to get too slow / run out of memory at this point
  dt_ = movement_duration_ / (num_time_steps_-1.0);

  for (unsigned int i=0; i<per_rollout_data_.size(); ++i)
  {
    per_rollout_data_[i].task_ = this;
    per_rollout_data_[i].cost_function_input_.resize(num_time_steps_);
    for (int t=0; t<num_time_steps_; ++t)
    {
      per_rollout_data_[i].cost_function_input_[t].reset(new StompCostFunctionInput(
          collision_space_, robot_model_, planning_group_));
    }
    per_rollout_data_[i].features_ = Eigen::MatrixXd(num_time_steps_, num_split_features_);

    per_rollout_data_[i].tmp_joint_angles_.resize(num_dimensions_, Eigen::VectorXd(num_time_steps_));
    per_rollout_data_[i].tmp_joint_angles_vel_.resize(num_dimensions_, Eigen::VectorXd(num_time_steps_));
    per_rollout_data_[i].tmp_joint_angles_acc_.resize(num_dimensions_, Eigen::VectorXd(num_time_steps_));
    std::vector<Eigen::VectorXd> v(3, Eigen::VectorXd(num_time_steps_));
    int nc = planning_group_->collision_points_.size();
    per_rollout_data_[i].tmp_collision_point_pos_.resize(nc, v);
    per_rollout_data_[i].tmp_collision_point_vel_.resize(nc, v);
    per_rollout_data_[i].tmp_collision_point_acc_.resize(nc, v);
    per_rollout_data_[i].fk_solver_ = planning_group_->getNewFKSolver();
  }

  // create the derivative costs
  std::vector<Eigen::MatrixXd> derivative_costs(num_dimensions_,
                                                Eigen::MatrixXd::Zero(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING, stomp::NUM_DIFF_RULES));
  std::vector<Eigen::VectorXd> initial_trajectory(num_dimensions_,
                                                  Eigen::VectorXd::Zero(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING));

  for (int d=0; d<num_dimensions_; ++d)
  {
    //derivative_costs[d].col(stomp::STOMP_VELOCITY) = Eigen::VectorXd::Ones(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    derivative_costs[d].col(stomp::STOMP_ACCELERATION) = Eigen::VectorXd::Ones(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    initial_trajectory[d] = Eigen::VectorXd::Zero(num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    initial_trajectory[d].head(stomp::TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING) * start[d];
    initial_trajectory[d].tail(stomp::TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING) * goal[d];
  }

  policy_.reset(new stomp::CovariantMovementPrimitive());
  policy_->initialize(num_time_steps_,
                      num_dimensions_,
                      movement_duration_,
                      derivative_costs,
                      initial_trajectory);
  policy_->setToMinControlCost();

  // set up feature basis functions
  std::vector<double> feature_split_magnitudes(num_feature_basis_functions_);
  feature_basis_functions_ = Eigen::MatrixXd::Zero(num_time_steps_, num_feature_basis_functions_);
  for (int t=0; t<num_time_steps_; ++t)
  {
    // compute feature splits and normalizations
    double p = double(t) / double(num_time_steps_-1);
    double sum = 0.0;
//    printf("t = %2d, ", t);
    for (int i=0; i<num_feature_basis_functions_; ++i)
    {
      feature_split_magnitudes[i] = (1.0 / (feature_basis_stddev_[i] * sqrt(2.0*M_PI))) *
          exp(-pow((p - feature_basis_centers_[i])/feature_basis_stddev_[i],2)/2.0);
      sum += feature_split_magnitudes[i];
    }
    for (int i=0; i<num_feature_basis_functions_; ++i)
    {
      feature_split_magnitudes[i] /= sum;
      feature_basis_functions_(t, i) = feature_split_magnitudes[i];
//      printf("f%d = %f, ", i, feature_split_magnitudes[i]);
    }
//    printf("\n");

  }
  //policy_->writeToFile(std::string("/tmp/test.txt"));

}

void StompOptimizationTask::parametersToJointTrajectory(const std::vector<Eigen::VectorXd>& parameters, trajectory_msgs::JointTrajectory& trajectory)
{
  trajectory.joint_names = planning_group_->getJointNames();

  trajectory.points.resize(num_time_steps_ + 2);
  trajectory.points[0].positions = start_joints_;
  trajectory.points[num_time_steps_+1].positions = goal_joints_;

  std::vector<Eigen::VectorXd> vels, accs;
  vels.resize(num_dimensions_);
  accs.resize(num_dimensions_);
  for (int d=0; d<num_dimensions_; ++d)
  {
    stomp::differentiate(parameters[d], stomp::STOMP_VELOCITY, vels[d], dt_);
    stomp::differentiate(parameters[d], stomp::STOMP_ACCELERATION, accs[d], dt_);
  }

  for (int i=0; i<num_time_steps_; ++i)
  {
    int j=i+1;
    trajectory.points[j].positions.resize(num_dimensions_);
    trajectory.points[j].velocities.resize(num_dimensions_);
    trajectory.points[j].accelerations.resize(num_dimensions_);
    for (int d=0; d<num_dimensions_; ++d)
    {
      trajectory.points[j].positions[d] = parameters[d](i);
      trajectory.points[j].velocities[d] = vels[d](i);
      trajectory.points[j].accelerations[d] = accs[d](i);
    }
  }

  for (int i=0; i<num_time_steps_+2; ++i)
  {
    trajectory.points[i].time_from_start = ros::Duration(i*dt_);
  }
}

void StompOptimizationTask::setFeatureWeights(const std::vector<double>& weights)
{
  ROS_ASSERT((int)weights.size() == num_split_features_);
  feature_weights_ = Eigen::VectorXd::Zero(weights.size());
  for (unsigned int i=0; i<weights.size(); ++i)
  {
    feature_weights_(i) = weights[i];
  }
}

void StompOptimizationTask::setFeatureWeights(const Eigen::VectorXd& weights)
{
  ROS_ASSERT(weights.rows() == feature_weights_.rows());
  feature_weights_ = weights;
}

void StompOptimizationTask::setFeatureWeightsFromFile(const std::string& abs_file_name)
{
  std::vector<double> weights;
  ROS_VERIFY(usc_utilities::readDoubleArrayFromFile(abs_file_name, weights));
  setFeatureWeights(weights);
}

void StompOptimizationTask::setFeatureScaling(const std::vector<double>& means, const std::vector<double>& variances)
{
  ROS_ASSERT((int)means.size() == num_split_features_);
  ROS_ASSERT((int)variances.size() == num_split_features_);
  feature_means_ = Eigen::VectorXd::Zero(means.size());
  feature_variances_ = Eigen::VectorXd::Zero(variances.size());
  for (int i=0; i<num_split_features_; ++i)
  {
    feature_means_(i) = means[i];
    feature_variances_(i) = variances[i];
  }
}

void StompOptimizationTask::setFeatureScalingFromFile(const std::string& abs_means_file,
                               const std::string& abs_variance_file)
{
  std::vector<double> means, variances;
  ROS_VERIFY(usc_utilities::readDoubleArrayFromFile(abs_means_file, means));
  ROS_VERIFY(usc_utilities::readDoubleArrayFromFile(abs_variance_file, variances));
  setFeatureScaling(means, variances);
}

void StompOptimizationTask::setInitialTrajectory(const std::vector<sensor_msgs::JointState>& joint_states)
{
  ROS_ASSERT((int)joint_states.size() == num_time_steps_);
  std::vector<Eigen::VectorXd> params(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
  for (int t=0; t<num_time_steps_; ++t)
  {
    for (unsigned int j=0; j<joint_states[t].name.size(); ++j)
    {
      for (unsigned int sj=0; sj<planning_group_->stomp_joints_.size(); ++sj)
      {
        if (joint_states[t].name[j] == planning_group_->stomp_joints_[sj].joint_name_)
        {
          params[sj](t) = joint_states[t].position[j];
        }
      }
    }
  }
  policy_->setParameters(params);
}

void StompOptimizationTask::getTrajectory(std::vector<sensor_msgs::JointState>& joint_states)
{
  std::vector<Eigen::VectorXd> params(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
  policy_->getParameters(params);
  joint_states.resize(num_time_steps_);
  ros::Time start_time = ros::Time::now();

  for (int t=0; t<num_time_steps_; ++t)
  {
    joint_states[t].header.stamp = start_time + ros::Duration(t*dt_);
    joint_states[t].name.resize(planning_group_->num_joints_);
    joint_states[t].position.resize(planning_group_->num_joints_);
    for (unsigned int sj=0; sj<planning_group_->stomp_joints_.size(); ++sj)
    {
      joint_states[t].name[sj] = planning_group_->stomp_joints_[sj].joint_name_;
      joint_states[t].position[sj] = params[sj](t);
    }
  }
}

void StompOptimizationTask::setToMinControlCostTrajectory()
{
  policy_->setToMinControlCost();
}

void StompOptimizationTask::getNoisyRolloutData(std::vector<PerRolloutData>& noisy_rollouts)
{
  noisy_rollouts.resize(num_rollouts_);
  for (int i=0; i<num_rollouts_; ++i)
    noisy_rollouts[i] = per_rollout_data_[i];
}

void StompOptimizationTask::getNoiselessRolloutData(PerRolloutData& noiseless_rollout)
{
  noiseless_rollout = per_rollout_data_[num_rollouts_];
}

void StompOptimizationTask::publishCollisionModelMarkers(int rollout_number)
{
  int rollout_id = num_rollouts_;
  if (rollout_number >= 0)
    rollout_id = rollout_number;
  PerRolloutData* data = &per_rollout_data_[rollout_id];

  // animate
  for (unsigned int i=0; i<data->cost_function_input_.size(); ++i)
  {
    data->cost_function_input_[i]->publishVizMarkers(ros::Time::now(), viz_pub_);
    ros::Duration(dt_).sleep();
  }
}

void StompOptimizationTask::publishTrajectoryMarkers(ros::Publisher& viz_pub)
{
  if (viz_pub.getNumSubscribers() == 0)
    return;
  for (int i=0; i<=last_executed_rollout_; ++i)
  {
    per_rollout_data_[i].publishMarkers(viz_pub, i, false, reference_frame_);
  }
  per_rollout_data_[num_rollouts_].publishMarkers(viz_pub, 0, true, reference_frame_);

  // publish empty markers to the remaining IDs
  for (int i=last_executed_rollout_+1; i<=max_rollout_markers_published_; ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "noisy";
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::DELETE;
    viz_pub.publish(marker);
  }

  if (max_rollout_markers_published_ < (int)last_executed_rollout_)
    max_rollout_markers_published_ = last_executed_rollout_;
}

void StompOptimizationTask::PerRolloutData::publishMarkers(ros::Publisher& viz_pub, int id, bool noiseless, const std::string& reference_frame)
{
  if (viz_pub.getNumSubscribers() == 0)
    return;

  std_msgs::ColorRGBA color;
  double size=0.0;
  std::string ns;
  if (noiseless)
  {
    color.a = 1.0;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 1.0;
    size = 0.02;
    ns="noiseless";
  }
  else
  {
    color.a = 0.5;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    size = 0.003;
    ns = "noisy";
  }

  publishMarkers(viz_pub, id, ns, color, size, reference_frame);
}

void StompOptimizationTask::PerRolloutData::publishMarkers(ros::Publisher& viz_pub, int id, const std::string& ns,
                                                           const std_msgs::ColorRGBA& color, double size,
                                                           const std::string& reference_frame)
{
  if (viz_pub.getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = reference_frame;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(cost_function_input_.size());
  marker.colors.resize(cost_function_input_.size());
  for (unsigned int t=0; t<cost_function_input_.size(); ++t)
  {
    KDL::Frame& v = cost_function_input_[t]->segment_frames_
        [task_->planning_group_->end_effector_segment_index_];
    marker.points[t].x = v.p.x();
    marker.points[t].y = v.p.y();
    marker.points[t].z = v.p.z();
    marker.colors[t] = color;
  }
  marker.scale.x = size;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  viz_pub.publish(marker);
}


void StompOptimizationTask::onEveryIteration()
{
  if (publish_trajectory_markers_)
    publishTrajectoryMarkers(viz_trajectory_pub_);
}

void StompOptimizationTask::setTrajectoryVizPublisher(ros::Publisher& viz_trajectory_pub)
{
  publish_trajectory_markers_ = true;
  viz_trajectory_pub_ = viz_trajectory_pub;
}

const StompRobotModel::StompPlanningGroup* StompOptimizationTask::getPlanningGroup()
{
  return planning_group_;
}

int StompOptimizationTask::getNumFeatures()
{
  return num_features_;
}

} /* namespace stomp_ros_interface */
