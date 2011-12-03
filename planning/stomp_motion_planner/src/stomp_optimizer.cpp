/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#include <stomp_motion_planner/stomp_optimizer.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <stomp_motion_planner/stomp_utils.h>
#include <Eigen/LU>


using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

namespace stomp_motion_planner
{

StompOptimizer::StompOptimizer(StompTrajectory *trajectory, const StompRobotModel *robot_model,
    const StompRobotModel::StompPlanningGroup *planning_group, const StompParameters *parameters,
    const ros::Publisher& vis_marker_array_publisher,
    const ros::Publisher& vis_marker_publisher,
    const ros::Publisher& stats_publisher,
    StompCollisionSpace *collision_space,
    const motion_planning_msgs::Constraints& constraints,
    planning_environment::PlanningMonitor* monitor):
      full_trajectory_(trajectory),
      robot_model_(robot_model),
      monitor_(monitor),
      planning_group_(planning_group),
      parameters_(parameters),
      collision_space_(collision_space),
      group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH),
      kdl_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
      kdl_vel_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
      kdl_acc_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
      kdl_group_joint_array_(group_trajectory_.getNumJoints()),
      kdl_group_vel_joint_array_(group_trajectory_.getNumJoints()),
      kdl_group_acc_joint_array_(group_trajectory_.getNumJoints()),
      kdl_group_torque_joint_array_(group_trajectory_.getNumJoints()),
      vis_marker_array_pub_(vis_marker_array_publisher),
      vis_marker_pub_(vis_marker_publisher),
      stats_pub_(stats_publisher),
      constraints_(constraints)
{
  initialize();
}

void StompOptimizer::initialize()
{

  //clearAnimations();

  // init some variables:
  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();

  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();


  //ROS_DEBUG_STREAM("Setting free vars start to " << free_vars_start_ << " end " << free_vars_end_);

  // set up joint index:
  group_joint_to_kdl_joint_index_.resize(num_joints_);
  for (int i=0; i<num_joints_; ++i)
    group_joint_to_kdl_joint_index_[i] = planning_group_->stomp_joints_[i].kdl_joint_index_;

  num_collision_points_ = planning_group_->collision_points_.size();

  // set up the joint costs:
  joint_costs_.reserve(num_joints_);

  double max_cost_scale = 0.0;
  ros::NodeHandle nh("~");
  for (int i=0; i<num_joints_; i++)
  {
    double joint_cost = 1.0;
    std::string joint_name = planning_group_->stomp_joints_[i].joint_name_;
    nh.param("joint_costs/"+joint_name, joint_cost, 1.0);
    std::vector<double> derivative_costs(3);
    derivative_costs[0] = joint_cost*parameters_->getSmoothnessCostVelocity();
    derivative_costs[1] = joint_cost*parameters_->getSmoothnessCostAcceleration();
    derivative_costs[2] = joint_cost*parameters_->getSmoothnessCostJerk();

    joint_costs_.push_back(StompCost(group_trajectory_, i, derivative_costs, parameters_->getRidgeFactor()));
    double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
    if (max_cost_scale < cost_scale)
      max_cost_scale = cost_scale;
  }

  // scale the smoothness costs
  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_[i].scale(max_cost_scale);
  }

  // allocate memory for matrices:
  smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  collision_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  final_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);
  jacobian_ = Eigen::MatrixXd::Zero(3, num_joints_);
  jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(num_joints_, 3);
  jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);
  random_state_ = Eigen::VectorXd::Zero(num_joints_);
  joint_state_velocities_ = Eigen::VectorXd::Zero(num_joints_);
  joint_state_accelerations_ = Eigen::VectorXd::Zero(num_joints_);
  full_joint_state_velocities_ = Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());
  full_joint_state_accelerations_ = Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());

  group_trajectory_backup_ = group_trajectory_.getTrajectory();
  best_group_trajectory_ = group_trajectory_.getTrajectory();

  joint_axis_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  joint_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  segment_frames_.resize(num_vars_all_, std::vector<KDL::Frame>(robot_model_->getKDLTree()->getNrOfSegments()));
  collision_point_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
  collision_point_vel_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
  collision_point_acc_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));

  collision_point_potential_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_vel_mag_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_potential_gradient_.resize(num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));

  // create the eigen maps:
  kdlVecVecToEigenVecVec(joint_axis_, joint_axis_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(joint_pos_, joint_pos_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_pos_, collision_point_pos_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_vel_, collision_point_vel_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_acc_, collision_point_acc_eigen_, 3, 1);

  collision_free_iteration_ = 0;
  is_collision_free_ = false;
  state_is_in_collision_.resize(num_vars_all_);
  point_is_in_collision_.resize(num_vars_all_, std::vector<int>(num_collision_points_));

  last_improvement_iteration_ = -1;

  // initialize exact collision checking stuff
  robot_state_.joint_state.name = robot_model_->getJointNames();
  robot_state_.joint_state.position.resize(robot_state_.joint_state.name.size());
  robot_state_.joint_state.velocity.resize(robot_state_.joint_state.name.size());
  //robot_state_.joint_state.effort.resize(robot_state_.joint_state.name.size());
  robot_state_.joint_state.header.frame_id = robot_model_->getReferenceFrame();
  allowed_contacts_ = monitor_->getAllowedContacts();
  kinematic_state_.reset(new planning_models::KinematicState(monitor_->getKinematicModel()));
  state_validity_.resize(num_vars_all_);

  // HMC initialization:
  momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_joint_momentum_ = Eigen::VectorXd::Zero(num_vars_free_);
  multivariate_gaussian_.clear();
  stochasticity_factor_ = 1.0;
  for (int i=0; i<num_joints_; i++)
  {
    multivariate_gaussian_.push_back(MultivariateGaussian(Eigen::VectorXd::Zero(num_vars_free_), joint_costs_[i].getQuadraticCostInverse()));
  }

  // animation init:
  animate_endeffector_segment_number_ = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(parameters_->getAnimateEndeffectorSegment());

  // initialize the policy
  policy_.reset(new CovariantTrajectoryPolicy());

  std::vector<double> derivative_costs = parameters_->getSmoothnessCosts();
  policy_->initialize(nh, num_vars_free_, num_joints_, group_trajectory_.getDuration(),
                      parameters_->getRidgeFactor(), derivative_costs);

  // initialize the policy trajectory
  Eigen::VectorXd start = group_trajectory_.getTrajectoryPoint(free_vars_start_-1).transpose();
  Eigen::VectorXd end = group_trajectory_.getTrajectoryPoint(free_vars_end_+1).transpose();
//  for (int i=0; i<start.rows(); ++i)
//  {
//    ROS_INFO("Start[%d] = %f\tEnd[%d] = %f", i, start(i), i, end(i));
//  }
  policy_->setToMinControlCost(start, end);

  // debug policy parameters:
//  std::vector<Eigen::VectorXd> policy_params;
//  policy_->getParameters(policy_params);
//  for (int i=0; i<policy_params[0].rows(); ++i)
//  {
//    printf("%f\n", policy_params[0][i]);
//  }

  // initialize the constraints:
  for (int i=0; i<int(constraints_.orientation_constraints.size()); ++i)
  {
    boost::shared_ptr<OrientationConstraintEvaluator> eval(new OrientationConstraintEvaluator(
        constraints_.orientation_constraints[i], *robot_model_));
    constraint_evaluators_.push_back(eval);
  }

  // initialize profiling
  mean_approx_collision_iteration_duration_ = 0.0;
  mean_exact_collision_iteration_duration_ = 0.0;
  num_approx_collision_iterations_ = 0;
  num_exact_collision_iterations_ = 0;

}

StompOptimizer::~StompOptimizer()
{
}

void StompOptimizer::doChompOptimization()
{
  calculateSmoothnessIncrements();
  calculateCollisionIncrements();
  calculateTotalIncrements();

  if (!parameters_->getUseHamiltonianMonteCarlo())
  {
    // non-stochastic version:
    addIncrementsToTrajectory();
  }
  else
  {
    // hamiltonian monte carlo updates:
    getRandomMomentum();
    updateMomentum();
    updatePositionFromMomentum();
    stochasticity_factor_ *= parameters_->getHmcAnnealingFactor();
  }

  handleJointLimits();
  updateFullTrajectory();
  last_trajectory_collision_free_ = performForwardKinematics();
  last_trajectory_constraints_satisfied_ = true;

  double cost = 0.0;
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    double cumulative = 0.0;
    for (int j=0; j<num_collision_points_; j++)
    {
      cumulative += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
      //state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
      state_collision_cost += cumulative;
    }
    cost += state_collision_cost * parameters_->getObstacleCostWeight();
  }
  last_trajectory_cost_ = cost;
}

void StompOptimizer::optimize()
{
  ros::WallTime start_time = ros::WallTime::now();

  state_ = STOMP_DISTANCE_FIELD_COLLISION;

  boost::shared_ptr<STOMPStatistics> stomp_statistics(new STOMPStatistics());

  stomp_statistics->collision_success_iteration = -1;
  stomp_statistics->success_iteration = -1;
  stomp_statistics->success = false;
  stomp_statistics->costs.clear();

  // initialize pi_loop
  ros::NodeHandle nh("~");
  PolicyImprovementLoop pi_loop;
  pi_loop.initialize(nh, this_shared_ptr_);
  

  collision_space_->lock();

  iteration_ = 0;
  copyPolicyToGroupTrajectory();
  handleJointLimits();
  updateFullTrajectory();
  performForwardKinematics();

  ROS_DEBUG("Trajectory cost: %f (s=%f, c=%f)", getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());

  if (parameters_->getAnimateEndeffector())
  {
    animateEndeffector();
  }

  if (parameters_->getAnimatePath())
  {
    animatePath();
  }

  // iterate
  for (iteration_=0; iteration_<parameters_->getMaxIterations(); iteration_++)
  {
    ros::WallTime iteration_start_time = ros::WallTime::now();
    if (!ros::ok())
      break;

    if (!parameters_->getUseChomp())
    {
      // after this, the latest "group trajectory" and "full trajectory" is the one optimized by pi^2
      //ros::WallTime start_time = ros::WallTime::now();
      pi_loop.runSingleIteration(iteration_+1);
      //ROS_INFO("PI loop took %f seconds, ", (ros::WallTime::now() - start_time).toSec());
    }
    else
    {
      doChompOptimization();
    }


    if (last_trajectory_collision_free_ && last_trajectory_constraints_satisfied_)
      collision_free_iteration_++;
    //else
    //  collision_free_iteration_ = 0;

    if (last_trajectory_collision_free_ &&
        stomp_statistics->collision_success_iteration == -1)
    {
      stomp_statistics->collision_success_iteration = iteration_;
      stomp_statistics->collision_success_duration = (ros::WallTime::now() - start_time).toSec();
    }
    if (last_trajectory_collision_free_ &&
        last_trajectory_constraints_satisfied_ &&
        stomp_statistics->success_iteration == -1)
    {
      stomp_statistics->success_iteration = iteration_;
      stomp_statistics->success = true;

      stomp_statistics->success_duration = (ros::WallTime::now() - start_time).toSec();
    }

//    double cost = getTrajectoryCost();
    double cost = last_trajectory_cost_;
    stomp_statistics->costs.push_back(cost);

    if (iteration_==0)
    {
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost_ = std::numeric_limits<double>::max();
    }
    else if (state_ == STOMP_EXACT_COLLISION)
    {
      if (cost < best_group_trajectory_cost_ && last_trajectory_collision_free_ && last_trajectory_constraints_satisfied_)
      {
        best_group_trajectory_ = group_trajectory_.getTrajectory();
        best_group_trajectory_cost_ = cost;
        last_improvement_iteration_ = iteration_;
      }
    }

    ROS_DEBUG("Trajectory cost: %f (s=%f, c=%f)", getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());

    double iteration_walltime = (ros::WallTime::now() - iteration_start_time).toSec();
    updateProfiling(iteration_walltime, state_==STOMP_EXACT_COLLISION);

    if (state_==STOMP_DISTANCE_FIELD_COLLISION &&
        ((last_trajectory_collision_free_ && collision_free_iteration_ == parameters_->getMaxIterationsAfterCollisionFree()-1)
      || iteration_ == parameters_->getMaxIterationsAfterCollisionFree()))
    {
      ROS_DEBUG("At iteration %d, switching to EXACT collision checking.", iteration_);
      state_ = STOMP_EXACT_COLLISION;
      pi_loop.clearReusedRollouts();
    }


    if (collision_free_iteration_ >= parameters_->getMaxIterationsAfterCollisionFree())
    {
      iteration_++;
      break;
    }

    if (parameters_->getAnimateEndeffector())
    {
      animateEndeffector();
    }

    if (parameters_->getAnimatePath() && iteration_%1 == 0)
    {
      animatePath();
    }
    


  }
  if (last_improvement_iteration_>-1)
    ROS_DEBUG_STREAM("We think the path is collision free: " << is_collision_free_);

  group_trajectory_.getTrajectory() = best_group_trajectory_;
  updateFullTrajectory();
  performForwardKinematics();
  if (parameters_->getAnimateEndeffector())
  {
    animateEndeffector();
  }
  if (parameters_->getAnimatePath())
  {
    animatePath();
  }

  collision_space_->unlock();

  ROS_INFO("STOMP terminated after %d iterations, using path from iteration %d", iteration_, last_improvement_iteration_);
  ROS_DEBUG("Best cost = %f", best_group_trajectory_cost_);
  ROS_DEBUG("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());
  ROS_DEBUG("Mean iteration durations: approx = %f msecs, exact = %f msecs",
            mean_approx_collision_iteration_duration_, mean_exact_collision_iteration_duration_);
  stomp_statistics->best_cost = best_group_trajectory_cost_;

  // calculate the torques for publishing
  std::vector<KDL::Wrench> wrenches(planning_group_->kdl_chain_.getNrOfSegments());
  for (unsigned int i=0; i<planning_group_->kdl_chain_.getNrOfSegments(); ++i)
  {
    wrenches[i].Zero();
  }
  std::vector<double> torques(num_joints_);
  stomp_statistics->torques.resize(num_vars_free_);
  for (int index = free_vars_start_; index <= free_vars_end_; ++index)
  {
    getTorques(index, torques, wrenches);
    stomp_statistics->torques[index-free_vars_start_] = 0.0;
    for (int j=0; j<num_joints_; ++j)
      stomp_statistics->torques[index-free_vars_start_] += fabs(torques[j]);
  }

  stats_pub_.publish(stomp_statistics);
}

void StompOptimizer::calculateSmoothnessIncrements()
{
  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i), smoothness_derivative_);
    smoothness_increments_.col(i) = -smoothness_derivative_.segment(
        group_trajectory_.getStartIndex(), num_vars_free_);
  }
}

void StompOptimizer::calculateCollisionIncrements()
{
  double potential;
  double vel_mag_sq;
  double vel_mag;
  Vector3d potential_gradient;
  Vector3d normalized_velocity;
  Matrix3d orthogonal_projector;
  Vector3d curvature_vector;
  Vector3d cartesian_gradient;

  collision_increments_.setZero(num_vars_free_, num_joints_);
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      potential = collision_point_potential_[i][j];
      if (potential <= 1e-10)
        continue;

      potential_gradient = collision_point_potential_gradient_[i][j];

      vel_mag = collision_point_vel_mag_[i][j];
      vel_mag_sq = vel_mag*vel_mag;

      // all math from the CHOMP paper:

      normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
      orthogonal_projector = Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
      curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
      cartesian_gradient = vel_mag*(orthogonal_projector*potential_gradient - potential*curvature_vector);

      // pass it through the jacobian transpose to get the increments
      planning_group_->collision_points_[j].getJacobian(joint_pos_eigen_[i], joint_axis_eigen_[i],
          collision_point_pos_eigen_[i][j], jacobian_, group_joint_to_kdl_joint_index_);
      if (parameters_->getUsePseudoInverse())
      {
        calculatePseudoInverse();
        collision_increments_.row(i-free_vars_start_).transpose() -=
            jacobian_pseudo_inverse_ * cartesian_gradient;
      }
      else
      {
        collision_increments_.row(i-free_vars_start_).transpose() -=
            jacobian_.transpose() * cartesian_gradient;
      }
      if (point_is_in_collision_[i][j])
        break;
    }
  }
  //cout << collision_increments_ << endl;
}

void StompOptimizer::calculatePseudoInverse()
{
  jacobian_jacobian_tranpose_ = jacobian_*jacobian_.transpose() + Eigen::MatrixXd::Identity(3,3)*parameters_->getPseudoInverseRidgeFactor();
  jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
}

void StompOptimizer::calculateTotalIncrements()
{
  for (int i=0; i<num_joints_; i++)
  {
    final_increments_.col(i) = parameters_->getLearningRate() *
        (
            joint_costs_[i].getQuadraticCostInverse() *
            (
                parameters_->getSmoothnessCostWeight() * smoothness_increments_.col(i) +
                parameters_->getObstacleCostWeight() * collision_increments_.col(i)
            )
        );
  }

}

void StompOptimizer::addIncrementsToTrajectory()
{
//  double scale = 1.0;
  for (int i=0; i<num_joints_; i++)
  {
    double scale = 1.0;
    double max = final_increments_.col(i).maxCoeff();
    double min = final_increments_.col(i).minCoeff();
    double max_scale = planning_group_->stomp_joints_[i].joint_update_limit_ / fabs(max);
    double min_scale = planning_group_->stomp_joints_[i].joint_update_limit_ / fabs(min);
    if (max_scale < scale)
      scale = max_scale;
    if (min_scale < scale)
      scale = min_scale;
    group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
  }
  //ROS_DEBUG("Scale: %f",scale);
  //group_trajectory_.getFreeTrajectoryBlock() += scale * final_increments_;
}

void StompOptimizer::updateFullTrajectory()
{
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void StompOptimizer::debugCost()
{
  double cost = 0.0;
  for (int i=0; i<num_joints_; i++)
    cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  cout << "Cost = " << cost << endl;
}

double StompOptimizer::getTrajectoryCost()
{
  return getSmoothnessCost() + getCollisionCost();
}

double StompOptimizer::getSmoothnessCost()
{
  double smoothness_cost = 0.0;
  // joint costs:
  for (int i=0; i<num_joints_; i++)
    smoothness_cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));

  return parameters_->getSmoothnessCostWeight() * smoothness_cost;
}

double StompOptimizer::getCollisionCost()
{
  double collision_cost = 0.0;

  double worst_collision_cost = 0.0;
  worst_collision_cost_state_ = -1;

  // collision costs:
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    for (int j=0; j<num_collision_points_; j++)
    {
      state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
    }
    collision_cost += state_collision_cost;
    if (state_collision_cost > worst_collision_cost)
    {
      worst_collision_cost = state_collision_cost;
      worst_collision_cost_state_ = i;
    }
  }

  return parameters_->getObstacleCostWeight() * collision_cost;
}

void StompOptimizer::handleJointLimits()
{
  for (int joint=0; joint<num_joints_; joint++)
  {
    if (!planning_group_->stomp_joints_[joint].has_joint_limits_)
      continue;

    double joint_max = planning_group_->stomp_joints_[joint].joint_limit_max_;
    double joint_min = planning_group_->stomp_joints_[joint].joint_limit_min_;

    int count = 0;

    bool violation = false;
    do
    {
      double max_abs_violation =  1e-6;
      double max_violation = 0.0;
      int max_violation_index = 0;
      violation = false;
      for (int i=free_vars_start_; i<=free_vars_end_; i++)
      {
        double amount = 0.0;
        double absolute_amount = 0.0;
        if (group_trajectory_(i, joint) > joint_max)
        {
          amount = joint_max - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        else if (group_trajectory_(i, joint) < joint_min)
        {
          amount = joint_min - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        if (absolute_amount > max_abs_violation)
        {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;
          violation = true;
        }
      }

      if (violation)
      {
        int free_var_index = max_violation_index - free_vars_start_;
        double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,free_var_index);
        group_trajectory_.getFreeJointTrajectoryBlock(joint) +=
            multiplier * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
      }
      if (++count > 10)
        break;
    }
    while(violation);
  }
}

bool StompOptimizer::performForwardKinematics()
{
  double invTime = 1.0 / group_trajectory_.getDiscretization();
  double invTimeSq = invTime*invTime;

  // calculate the forward kinematics for the fixed states only in the first iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_==0)
  {
    start = 0;
    end = num_vars_all_-1;
  }

  is_collision_free_ = true;
  int num_ignore_points = 0.01 * parameters_->getIgnoreStateValidityPercent() * num_vars_free_;

  // for each point in the trajectory
  for (int i=start; i<=end; ++i)
  {
    int full_traj_index = group_trajectory_.getFullTrajectoryIndex(i);
    full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);

    //ROS_INFO_STREAM("Trajectory point " << i << " index " << full_traj_index);
//     for(unsigned int j = 0; j < kdl_joint_array_.rows(); j++) {
//       ROS_INFO_STREAM(j << " " << kdl_joint_array_(j));
//     }

    if (iteration_==0)
    {
      planning_group_->fk_solver_->JntToCartFull(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);
    }
    else
    {
      planning_group_->fk_solver_->JntToCartPartial(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);
    }

    //robot_model_->getForwardKinematicsSolver()->JntToCart(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);

    state_is_in_collision_[i] = false;

    // calculate the position of every collision point:
    for (int j=0; j<num_collision_points_; j++)
    {
      planning_group_->collision_points_[j].getTransformedPosition(segment_frames_[i], collision_point_pos_[i][j]);
      //int segment_number = planning_group_->collision_points_[j].getSegmentNumber();
      //collision_point_pos_[i][j] = segment_frames_[i][segment_number] * planning_group_->collision_points_[j].getPosition();

      bool colliding = collision_space_->getCollisionPointPotentialGradient(planning_group_->collision_points_[j],
          collision_point_pos_eigen_[i][j],
          collision_point_potential_[i][j],
          collision_point_potential_gradient_[i][j]);

      point_is_in_collision_[i][j] = colliding;

      if (colliding)
        state_is_in_collision_[i] = true;
    }

    if (!(i - free_vars_start_ < num_ignore_points ||
        free_vars_end_ - i < num_ignore_points) &&
        state_is_in_collision_[i])
    {
      is_collision_free_ = false;
    }
    state_validity_[i] = !state_is_in_collision_[i];
  }

  // now, get the vel and acc for each collision point (using finite differencing)
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      SetToZero(collision_point_vel_[i][j]);
      SetToZero(collision_point_acc_[i][j]);
      for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
      {
        collision_point_vel_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) *
            collision_point_pos_[i+k][j];
        collision_point_acc_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) *
            collision_point_pos_[i+k][j];
      }
      // get the norm of the velocity:
      collision_point_vel_mag_[i][j] = collision_point_vel_eigen_[i][j].norm();
    }


  }

//  if (is_collision_free_)
//    collision_free_iteration_++;
//  else
//    collision_free_iteration_ = 0;

  return is_collision_free_;
}

void StompOptimizer::eigenMapTest()
{
  double foo_eigen;
  double foo_kdl;

  cout << "Eigen location: " << &(joint_axis_eigen_[free_vars_start_][0](0)) <<
          "  KDL location: " << &(joint_axis_[free_vars_start_][0](0)) << endl;

  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);

  joint_axis_eigen_[free_vars_start_][0](0) = 1.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_kdl == foo_eigen);

  joint_axis_[free_vars_start_][0](0) = 2.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen == foo_kdl);

  foo_eigen = joint_pos_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_pos_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);

  foo_eigen = collision_point_pos_eigen_[free_vars_start_][5](0);
  foo_kdl = collision_point_pos_[free_vars_start_][5](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);
}

void StompOptimizer::perturbTrajectory()
{
  //int mid_point = (free_vars_start_ + free_vars_end_) / 2;
  if (worst_collision_cost_state_ < 0)
    return;
  int mid_point = worst_collision_cost_state_;
  planning_group_->getRandomState(random_state_);

  // convert the state into an increment
  random_state_ -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();

  // project the increment orthogonal to joint velocities
  group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
  joint_state_velocities_.normalize();
  random_state_ = (Eigen::MatrixXd::Identity(num_joints_, num_joints_) - joint_state_velocities_*joint_state_velocities_.transpose()) * random_state_;

  int mp_free_vars_index = mid_point - free_vars_start_;
  for (int i=0; i<num_joints_; i++)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(i) +=
        joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) * random_state_(i);
  }
}

void StompOptimizer::getRandomMomentum()
{
  if (is_collision_free_)
    random_momentum_.setZero(num_vars_free_, num_joints_);
  else
  for (int i=0; i<num_joints_; ++i)
  {
    multivariate_gaussian_[i].sample(random_joint_momentum_);
    random_momentum_.col(i) = stochasticity_factor_ * random_joint_momentum_;
  }
}

void StompOptimizer::updateMomentum()
{
  double eps = parameters_->getHmcDiscretization();

  //  if (iteration_ > 0)
//    momentum_ = (momentum_ + eps*final_increments_);
//  else
//    momentum_ = random_momentum_;

  double alpha = 1.0 - parameters_->getHmcStochasticity();
  //if (iteration_ > 0)
    momentum_ = alpha * (momentum_ + eps*final_increments_) + sqrt(1.0-alpha*alpha)*random_momentum_;
  //else
  //  momentum_ = random_momentum_;
}

void StompOptimizer::updatePositionFromMomentum()
{
  double eps = parameters_->getHmcDiscretization();
  group_trajectory_.getFreeTrajectoryBlock() += eps * momentum_;
}

void StompOptimizer::animatePath()
{
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    visualizeState(i);
    //ros::WallDuration(group_trajectory_.getDiscretization()).sleep();
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }
}

void StompOptimizer::clearAnimations()
{
  visualization_msgs::Marker msg;
  msg.points.resize(0);
  msg.header.frame_id = robot_model_->getReferenceFrame();
  msg.header.stamp = ros::Time::now();
  msg.ns = "stomp_endeffector";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::Marker::DELETE;
  vis_marker_pub_.publish(msg);
}

void StompOptimizer::animateEndeffector()
{
  visualization_msgs::Marker msg;
  msg.header.frame_id = robot_model_->getReferenceFrame();
  msg.header.stamp = ros::Time::now();
  msg.ns = "stomp_endeffector";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  double scale = 0.05;
  msg.scale.x = scale;
  msg.scale.y = scale;
  msg.scale.z = scale;
  msg.color.a = 1.0;
  msg.color.r = 0.1;
  msg.color.g = 0.1;
  msg.color.b = 1.0;

  visualization_msgs::Marker msg_bad = msg;
  msg_bad.id = 1;
  msg_bad.color.r = 1.0;
  msg_bad.color.b = 0.1;

  //msg.points.resize(num_vars_free_);
 // int joint_index = planning_group_->stomp_joints_[num_joints_-1].kdl_joint_index_;
  int sn = animate_endeffector_segment_number_;
  for (int i=0; i<num_vars_free_; ++i)
  {
    int j = i+free_vars_start_;
    geometry_msgs::Point point;
    point.x = segment_frames_[j][sn].p.x();
    point.y = segment_frames_[j][sn].p.y();
    point.z = segment_frames_[j][sn].p.z();
    if (state_validity_[j])
      msg.points.push_back(point);
    else
      msg_bad.points.push_back(point);
  }
  vis_marker_pub_.publish(msg);
  vis_marker_pub_.publish(msg_bad);


//  visualization_msgs::Marker msg2;
//  msg2.points.resize(num_vars_free_*2);
//  for (int i=0; i<num_vars_free_; ++i)
//  {
//    int j = i+free_vars_start_;
//    int start = i*2;
//    int end = start + 1;
//    msg2.points[start].x = segment_frames_[j][sn].p.x();
//    msg2.points[start].y = segment_frames_[j][sn].p.y();
//    msg2.points[start].z = segment_frames_[j][sn].p.z();
//
//    KDL::Vector v(0.0,0.0,0.2);
//    v = segment_frames_[j][sn].M*v;
//
//    msg2.points[end].x = segment_frames_[j][sn].p.x() + v.x();
//    msg2.points[end].y = segment_frames_[j][sn].p.y() + v.y();
//    msg2.points[end].z = segment_frames_[j][sn].p.z() + v.z();
//  }
//  msg2.pose.position.x = 0.0;
//  msg2.pose.position.y = 0.0;
//  msg2.pose.position.z = 0.0;
//  msg2.pose.orientation.w = 1.0;
//  msg2.pose.orientation.x = 0.0;
//  msg2.pose.orientation.y = 0.0;
//  msg2.pose.orientation.z = 0.0;
//  msg2.header.frame_id = robot_model_->getReferenceFrame();
//  msg2.header.stamp = ros::Time::now();
//  msg2.ns = "stomp_endeffector_arrows";
//  msg2.id = 0;
//  msg2.type = visualization_msgs::Marker::LINE_LIST;
//  msg2.action = visualization_msgs::Marker::ADD;
//  double line_width = 0.02;
//  msg2.scale.x = line_width;
//  msg2.color.a = 0.6;
//  msg2.color.r = 0.5;
//  msg2.color.g = 1.0;
//  msg2.color.b = 0.3;
//  vis_marker_pub_.publish(msg2);

  ros::spinOnce();
  ros::WallDuration(0.001).sleep();
  //char c;
  //std::cin.get(c);
}

void StompOptimizer::visualizeState(int index)
{
  visualization_msgs::MarkerArray msg;
//  msg.markers.resize(num_collision_points_ + num_joints_);
  msg.markers.resize(num_collision_points_);
  int num_arrows = 0;
  double potential_threshold = 1e-10;
  for (int i=0; i<num_collision_points_; i++)
  {
    msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
    msg.markers[i].header.stamp = ros::Time();
    msg.markers[i].ns = "stomp_collisions";
    msg.markers[i].id = i;
    msg.markers[i].type = visualization_msgs::Marker::SPHERE;
    msg.markers[i].action = visualization_msgs::Marker::ADD;
    msg.markers[i].pose.position.x = collision_point_pos_[index][i].x();
    msg.markers[i].pose.position.y = collision_point_pos_[index][i].y();
    msg.markers[i].pose.position.z = collision_point_pos_[index][i].z();
    msg.markers[i].pose.orientation.x = 0.0;
    msg.markers[i].pose.orientation.y = 0.0;
    msg.markers[i].pose.orientation.z = 0.0;
    msg.markers[i].pose.orientation.w = 1.0;
    double scale = planning_group_->collision_points_[i].getRadius()*2.0;
    msg.markers[i].scale.x = scale;
    msg.markers[i].scale.y = scale;
    msg.markers[i].scale.z = scale;
    msg.markers[i].color.a = 0.6;
    msg.markers[i].color.r = 0.5;
    msg.markers[i].color.g = 1.0;
    msg.markers[i].color.b = 0.3;
    if (collision_point_potential_[index][i] > potential_threshold)
      num_arrows++;
  }

//  for (int j=0; j<num_joints_; ++j)
//  {
//    int i=num_collision_points_+j;
//    int joint_index = planning_group_->stomp_joints_[j].kdl_joint_index_;
//    msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
//    msg.markers[i].header.stamp = ros::Time();
//    msg.markers[i].ns = "stomp_collisions";
//    msg.markers[i].id = i;
//    msg.markers[i].type = visualization_msgs::Marker::ARROW;
//    msg.markers[i].action = visualization_msgs::Marker::ADD;
//    msg.markers[i].points.resize(2);
//    double scale=0.2;
//    double sx = joint_axis_[index][joint_index].x()*scale;
//    double sy = joint_axis_[index][joint_index].y()*scale;
//    double sz = joint_axis_[index][joint_index].z()*scale;
//    msg.markers[i].points[0].x = joint_pos_[index][joint_index].x() - sx;
//    msg.markers[i].points[0].y = joint_pos_[index][joint_index].y() - sy;
//    msg.markers[i].points[0].z = joint_pos_[index][joint_index].z() - sz;
//    msg.markers[i].points[1].x = joint_pos_[index][joint_index].x() + sx;
//    msg.markers[i].points[1].y = joint_pos_[index][joint_index].y() + sy;
//    msg.markers[i].points[1].z = joint_pos_[index][joint_index].z() + sz;
//    msg.markers[i].scale.x = 0.03;
//    msg.markers[i].scale.y = 0.06;
//    msg.markers[i].color.a = 1.0;
//    msg.markers[i].color.r = 1.0;
//    msg.markers[i].color.g = 0.5;
//    msg.markers[i].color.b = 0.5;
//  }
  vis_marker_array_pub_.publish(msg);

  // publish arrows for distance field:
  msg.markers.resize(0);
  msg.markers.resize(num_collision_points_);
  for (int i=0; i<num_collision_points_; i++)
  {
      msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
      msg.markers[i].header.stamp = ros::Time();
      msg.markers[i].ns = "stomp_arrows";
      msg.markers[i].id = i;
      msg.markers[i].type = visualization_msgs::Marker::ARROW;
      msg.markers[i].action = visualization_msgs::Marker::ADD;
      msg.markers[i].points.resize(2);
      msg.markers[i].points[0].x = collision_point_pos_[index][i].x();
      msg.markers[i].points[0].y = collision_point_pos_[index][i].y();
      msg.markers[i].points[0].z = collision_point_pos_[index][i].z();
      msg.markers[i].points[1] = msg.markers[i].points[0];
      double scale = 0.15;
      if (collision_point_potential_[index][i] <= potential_threshold)
        scale = 0.0;
      msg.markers[i].points[1].x -= scale*collision_point_potential_gradient_[index][i].x();
      msg.markers[i].points[1].y -= scale*collision_point_potential_gradient_[index][i].y();
      msg.markers[i].points[1].z -= scale*collision_point_potential_gradient_[index][i].z();
      msg.markers[i].scale.x = 0.01;
      msg.markers[i].scale.y = 0.03;
      msg.markers[i].color.a = 1.0;
      msg.markers[i].color.r = 0.2;
      msg.markers[i].color.g = 0.2;
      msg.markers[i].color.b = 1.0;
  }
  vis_marker_array_pub_.publish(msg);
}


bool StompOptimizer::initialize(ros::NodeHandle& node_handle, int num_time_steps)
{
  // we already know these things, so do nothing
  return true;
}

void StompOptimizer::getTorques(int index, std::vector<double>& torques, const std::vector<KDL::Wrench>& wrenches)
{

  int i=index;
  group_trajectory_.getTrajectoryPointKDL(i, kdl_group_joint_array_);
  group_trajectory_.getJointVelocities(i, joint_state_velocities_);
  group_trajectory_.getJointAccelerations(i, joint_state_accelerations_);
  for (int j=0; j<num_joints_; ++j)
  {
    //kdl_group_joint_array_(j) = kdl_joint_array_(full_joint_num);
    kdl_group_vel_joint_array_(j) = joint_state_velocities_(j);
    kdl_group_acc_joint_array_(j) = joint_state_accelerations_(j);
  }
  planning_group_->id_solver_->CartToJnt(kdl_group_joint_array_,
                                         kdl_group_vel_joint_array_,
                                         kdl_group_acc_joint_array_,
                                         wrenches,
                                         kdl_group_torque_joint_array_);

  for (int j=0; j<num_joints_; ++j)
  {
    torques[j] = kdl_group_torque_joint_array_(j);
  }

}

bool StompOptimizer::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number)
{
  //costs = Eigen::VectorXd::Zero(num_vars_free_);
  //return true;
  ros::WallTime start_time = ros::WallTime::now();

  // copy the parameters into group_trajectory_:
  for (int d=0; d<num_joints_; ++d)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d];
  }

  // respect joint limits:
  handleJointLimits();

  // copy to full traj:
  updateFullTrajectory();

  // do forward kinematics:
  last_trajectory_collision_free_ = performForwardKinematics();

  if (parameters_->getStateValidityCostWeight() > 1e-10 && state_ == STOMP_EXACT_COLLISION)
  {
    computeTrajectoryValidity();
    last_trajectory_collision_free_ = trajectory_validity_;
  }

  //animateEndeffector();

  last_trajectory_constraints_satisfied_ = true;

  double constraint_cost = 0.0;
  double obstacle_cost = 0.0;
  double torque_cost = 0.0;
  double validity_cost = 0.0;
  double endeffector_velocity_cost = 0.0;

  std::vector<KDL::Wrench> wrenches(planning_group_->kdl_chain_.getNrOfSegments());
  for (unsigned int i=0; i<planning_group_->kdl_chain_.getNrOfSegments(); ++i)
  {
    wrenches[i].Zero();
  }

  std::vector<double> torques(num_joints_);
  int sn = animate_endeffector_segment_number_;
  double total_dx = segment_frames_[free_vars_end_+1][sn].p.x() - segment_frames_[free_vars_start_-1][sn].p.x();
  double total_dy = segment_frames_[free_vars_end_+1][sn].p.y() - segment_frames_[free_vars_start_-1][sn].p.y();
  double total_dz = segment_frames_[free_vars_end_+1][sn].p.z() - segment_frames_[free_vars_start_-1][sn].p.z();
  double total_endeff_dist = sqrt(total_dx*total_dx + total_dy*total_dy + total_dz*total_dz);
  if (total_endeff_dist < 1e-2)
	  total_endeff_dist = 1e-2;

  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    //double cumulative = 0.0;
    for (int j=0; j<num_collision_points_; j++)
    {
      //cumulative += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
      //state_collision_cost += cumulative;

      state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
    }

    // evaluate the constraints:
    double state_constraint_cost = 0.0;
    for (int j=0; j<int(constraint_evaluators_.size()); ++j)
    {
      double cost;
      if (!constraint_evaluators_[j]->getCost(segment_frames_[i], full_trajectory_->getTrajectoryPoint(i), cost))
        last_trajectory_constraints_satisfied_ = false;
      state_constraint_cost += cost;
    }

    // evaluate inverse dynamics:
    double state_torque_cost = 0.0;

    if (parameters_->getTorqueCostWeight() > 1e-9)
    {
      getTorques(i, torques, wrenches);
      for (int j=0; j<num_joints_; ++j)
      {
        state_torque_cost += fabs(torques[j]);
      }
    }

    // exact collision checking:
    double state_validity_cost = 0.0;
    if (state_ == STOMP_EXACT_COLLISION &&
        !state_validity_[i])
      state_validity_cost = 1.0;

    double state_endeffector_velocity_cost = 0.0;
    double dx = segment_frames_[i][sn].p.x() - segment_frames_[i-1][sn].p.x();
    double dy = segment_frames_[i][sn].p.y() - segment_frames_[i-1][sn].p.y();
    double dz = segment_frames_[i][sn].p.z() - segment_frames_[i-1][sn].p.z();
    state_endeffector_velocity_cost = sqrt(dx*dx + dy*dy + dz*dz)/total_endeff_dist;

    obstacle_cost += parameters_->getObstacleCostWeight() * state_collision_cost;
    constraint_cost += parameters_->getConstraintCostWeight() * state_constraint_cost;
    torque_cost += parameters_->getTorqueCostWeight() * state_torque_cost;
    validity_cost += parameters_->getStateValidityCostWeight() * state_validity_cost;
    endeffector_velocity_cost += parameters_->getEndeffectorVelocityCostWeight() * state_endeffector_velocity_cost;

    costs(i-free_vars_start_) =
        parameters_->getObstacleCostWeight() * state_collision_cost +
        parameters_->getConstraintCostWeight() * state_constraint_cost +
        parameters_->getTorqueCostWeight() * state_torque_cost +
        parameters_->getStateValidityCostWeight() * state_validity_cost +
        parameters_->getEndeffectorVelocityCostWeight() * state_endeffector_velocity_cost;
  }

  last_trajectory_cost_ = costs.sum();
  //last_trajectory_constraints_satisfied_ = (constraint_cost < 1e-6);

  //ROS_INFO("Obstacle cost = %f, constraint cost = %f, torque_cost = %f, validity_cost = %f", obstacle_cost, constraint_cost, torque_cost, validity_cost);
  //animateEndeffector();
  //ros::spinOnce();
  //ros::Duration(1.0).sleep();

  //ROS_INFO("Rollout took %f seconds, ", (ros::WallTime::now() - start_time).toSec());
  return true;
}

void StompOptimizer::computeTrajectoryValidity()
{
  trajectory_validity_ = true;
  int num_ignore_points = 0.01 * parameters_->getIgnoreStateValidityPercent() * num_vars_free_;
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    state_validity_[i] = true;
    if (i - free_vars_start_ < num_ignore_points ||
        free_vars_end_ - i < num_ignore_points)
      continue;

    int full_traj_index = group_trajectory_.getFullTrajectoryIndex(i);
    full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);
    for (int j=0; j<full_trajectory_->getNumJoints(); ++j)
    {
      robot_state_.joint_state.position[j] = kdl_joint_array_(j);
    }
    monitor_->setRobotStateAndComputeTransforms(robot_state_, *kinematic_state_);
    monitor_->getEnvironmentModel()->updateRobotModel(&(*kinematic_state_));
    state_validity_[i] = !monitor_->getEnvironmentModel()->getCollisionContacts(allowed_contacts_, contacts_, 1);
    if (!state_validity_[i])
      trajectory_validity_ = false;
  }

}

bool StompOptimizer::getPolicy(boost::shared_ptr<stomp_motion_planner::Policy>& policy)
{
  policy = policy_;
  return true;
}

bool StompOptimizer::setPolicy(const boost::shared_ptr<stomp_motion_planner::Policy> policy)
{
  return true;
}

bool StompOptimizer::getControlCostWeight(double& control_cost_weight)
{
  control_cost_weight = parameters_->getSmoothnessCostWeight();
  return true;
}

void StompOptimizer::copyPolicyToGroupTrajectory()
{
  policy_->getParameters(policy_parameters_);
  for (int d=0; d<num_joints_; ++d)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(d) = policy_parameters_[d];
  }
}

void StompOptimizer::copyGroupTrajectoryToPolicy()
{
  for (int d=0; d<num_joints_; ++d)
  {
    policy_parameters_[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
  }
  policy_->setParameters(policy_parameters_);
}

void StompOptimizer::setSharedPtr(boost::shared_ptr<StompOptimizer>& ptr)
{
  this_shared_ptr_ = ptr;
}

void StompOptimizer::resetSharedPtr()
{
  this_shared_ptr_.reset();
}


void StompOptimizer::updateProfiling(double iteration_duration, bool exact_collision_checking)
{
  double& mean = (exact_collision_checking) ? mean_exact_collision_iteration_duration_ : mean_approx_collision_iteration_duration_;
  int& num = (exact_collision_checking) ? num_exact_collision_iterations_ : num_approx_collision_iterations_;

  mean = ((mean * num) + iteration_duration)/double(++num);
}

} // namespace stomp
