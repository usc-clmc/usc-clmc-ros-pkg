/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <policy_library/covariant_trajectory_policy.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/Core>
#include <sstream>
#include <tf/transform_datatypes.h>

using namespace Eigen;


namespace policy_library
{

static const char* CART_POSE_NAMES[]= {
    "CART_X",
    "CART_Y",
    "CART_Z",
    "CART_QW",
    "CART_QX",
    "CART_QY",
    "CART_QZ"
};
static const char* CART_WRENCH_NAMES[]= {
    "FORCE_X",
    "FORCE_Y",
    "FORCE_Z",
    "TORQUE_X",
    "TORQUE_Y",
    "TORQUE_Z"
};

const char* CovariantTrajectoryPolicy::TOPIC_NAME="/CovariantTrajectoryPolicy";

CovariantTrajectoryPolicy::CovariantTrajectoryPolicy()
{
}

CovariantTrajectoryPolicy::~CovariantTrajectoryPolicy()
{
}

bool CovariantTrajectoryPolicy::initialize(ros::NodeHandle node_handle)
{
    node_handle_ = node_handle;

    ROS_VERIFY(readParameters());
    ROS_VERIFY(initializeVariables());
    ROS_VERIFY(initializeCosts());
    ROS_VERIFY(initializeBasisFunctions());

    return true;
}

bool CovariantTrajectoryPolicy::initialize(ros::NodeHandle node_handle,
                                           const int num_time_steps,
                                           const int num_dimensions,
                                           const double movement_duration,
                                           const double cost_ridge_factor,
                                           const std::vector<double>& derivative_costs)
{
    node_handle_ = node_handle;

    num_time_steps_ = num_time_steps;
    num_dimensions_ = num_dimensions;
    movement_duration_ = movement_duration;
    cost_ridge_factor_ = cost_ridge_factor;
    derivative_costs_ = derivative_costs;

    dimension_names_.resize(num_dimensions);

    ROS_VERIFY(initializeVariables());
    ROS_VERIFY(initializeCosts());
    ROS_VERIFY(initializeBasisFunctions());

    return true;
}

bool CovariantTrajectoryPolicy::readParameters()
{
    node_handle_.param("num_time_steps", num_time_steps_, 100);
    node_handle_.param("num_dimensions", num_dimensions_, 1);
    node_handle_.param("movement_duration", movement_duration_, 1.0);
    node_handle_.param("cost_ridge_factor", cost_ridge_factor_, 0.00001);
    ROS_VERIFY(usc_utilities::readDoubleArray(node_handle_, "derivative_costs", derivative_costs_));
    return true;
}

void CovariantTrajectoryPolicy::readCostFunctionFromParamServer(ros::NodeHandle& node_handle)
{
  node_handle.param("cost_ridge_factor", cost_ridge_factor_, 0.00001);
  ROS_VERIFY(usc_utilities::readDoubleArray(node_handle, "derivative_costs", derivative_costs_));
  ROS_VERIFY(initializeCosts());
}

bool CovariantTrajectoryPolicy::setToMinControlCost(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
    for (int d=0; d<num_dimensions_; ++d)
    {
        // set the start and end of the trajectory
        for (int i=0; i<DIFF_RULE_LENGTH-1; ++i)
        {
            parameters_all_[d](i) = start(d);
            parameters_all_[d](num_vars_all_-1-i) = goal(d);
        }

    }
    computeLinearControlCosts();
    computeMinControlCostParameters();
    return true;
}

bool CovariantTrajectoryPolicy::computeLinearControlCosts()
{
    linear_control_costs_.clear();
    linear_control_costs_.resize(num_dimensions_, VectorXd::Zero(num_vars_free_));
    for (int d=0; d<num_dimensions_; ++d)
    {
        linear_control_costs_[d].transpose() = parameters_all_[d].segment(0, DIFF_RULE_LENGTH-1).transpose() *
                control_costs_all_[d].block(0, free_vars_start_index_, DIFF_RULE_LENGTH-1, num_vars_free_);
        linear_control_costs_[d].transpose() += parameters_all_[d].segment(free_vars_end_index_+1, DIFF_RULE_LENGTH-1).transpose() *
                control_costs_all_[d].block(free_vars_end_index_+1, free_vars_start_index_, DIFF_RULE_LENGTH-1, num_vars_free_);
        linear_control_costs_[d] *= 2.0;
    }
    return true;
}

bool CovariantTrajectoryPolicy::computeMinControlCostParameters()
{
    for (int d=0; d<num_dimensions_; ++d)
    {
        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) =
                -0.5 * inv_control_costs_[d] * linear_control_costs_[d];
    }
//    for (int d=0; d<num_dimensions_; ++d)
//    {
//        VectorXd gradient =  2.0 * control_costs_all_[d] * parameters_all_[d];
//        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_)
//                += -0.5 * inv_control_costs_[d] * gradient.segment(free_vars_start_index_, num_vars_free_);
//    }
    return true;
}

bool CovariantTrajectoryPolicy::initializeVariables()
{
    movement_dt_ = movement_duration_ / (num_time_steps_ + 1);

    num_vars_free_ = num_time_steps_;
    num_vars_all_ = num_vars_free_ + 2*(DIFF_RULE_LENGTH-1);
    free_vars_start_index_ = DIFF_RULE_LENGTH - 1;
    free_vars_end_index_ = free_vars_start_index_ + num_vars_free_ - 1;

    num_parameters_.clear();
    for (int d=0; d<num_dimensions_; ++d)
        num_parameters_.push_back(num_time_steps_);

    parameters_all_.resize(num_dimensions_, Eigen::VectorXd::Zero(num_vars_all_));

    return true;
}

bool CovariantTrajectoryPolicy::initializeCosts()
{
    createDifferentiationMatrices();

    control_costs_all_.clear();
    control_costs_.clear();
    inv_control_costs_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
        // construct the quadratic cost matrices (for all variables)
        MatrixXd cost_all = MatrixXd::Identity(num_vars_all_, num_vars_all_) * cost_ridge_factor_;
        for (int i=0; i<NUM_DIFF_RULES; ++i)
        {
            cost_all += derivative_costs_[i] * (differentiation_matrices_[i].transpose() * differentiation_matrices_[i]);
        }
        control_costs_all_.push_back(cost_all);

        // extract the quadratic cost just for the free variables:
        MatrixXd cost_free = cost_all.block(DIFF_RULE_LENGTH-1, DIFF_RULE_LENGTH-1, num_vars_free_, num_vars_free_);
        control_costs_.push_back(cost_free);
        inv_control_costs_.push_back(cost_free.inverse());
    }
    return true;
}

bool CovariantTrajectoryPolicy::initializeBasisFunctions()
{
    basis_functions_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
        basis_functions_.push_back(MatrixXd::Identity(num_vars_free_, num_vars_free_));
    }
    return true;
}


void CovariantTrajectoryPolicy::createDifferentiationMatrices()
{
    double multiplier = 1.0;
    differentiation_matrices_.clear();
    differentiation_matrices_.resize(NUM_DIFF_RULES, MatrixXd::Zero(num_vars_all_, num_vars_all_));
    for (int d=0; d<NUM_DIFF_RULES; ++d)
    {
        multiplier /= movement_dt_;
        for (int i=0; i<num_vars_all_; i++)
        {
            for (int j=-DIFF_RULE_LENGTH/2; j<=DIFF_RULE_LENGTH/2; j++)
            {
                int index = i+j;
                if (index < 0)
                    continue;
                if (index >= num_vars_all_)
                    continue;
                differentiation_matrices_[d](i,index) = multiplier * DIFF_RULES[d][j+DIFF_RULE_LENGTH/2];
            }
        }
        //ROS_INFO_STREAM(differentiation_matrices_[d]);
    }
}

bool CovariantTrajectoryPolicy::computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                 const double weight, std::vector<Eigen::VectorXd>& control_costs)
{
    //Policy::computeControlCosts(control_cost_matrices, parameters, weight, control_costs);

    // we use the locally stored control costs

    // this uses the already squared control cost matrix
    /*for (int d=0; d<num_dimensions_; ++d)
    {
        control_costs[d] = VectorXd::Zero(num_time_steps_);
        VectorXd params_all = parameters_all_[d];
        for (int t=0; t<num_time_steps_; ++t)
        {
            params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d][t];
            VectorXd r_times_u = control_costs_all_[d] * params_all;
            control_costs[d] += weight * (r_times_u.segment(free_vars_start_index_, num_vars_free_).cwise() * parameters[d][t]);
        }
    }*/


    // this measures the accelerations and squares them
    for (int d=0; d<num_dimensions_; ++d)
    {
        VectorXd params_all = parameters_all_[d];
        VectorXd costs_all = VectorXd::Zero(num_vars_all_);
        for (int t=0; t<num_time_steps_; ++t)
        {
            params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d][t];
            VectorXd acc_all = VectorXd::Zero(num_vars_all_);
            for (int i=0; i<NUM_DIFF_RULES; ++i)
            {
                acc_all = differentiation_matrices_[i]*params_all;
								// Eigen2
                // costs_all += weight * derivative_costs_[i] * (acc_all.cwise()*acc_all);
								// Eigen3
								costs_all += weight * derivative_costs_[i] * (acc_all.cwiseProduct(acc_all));
            }
        }
        control_costs[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
        for (int i=0; i<free_vars_start_index_; ++i)
        {
            control_costs[d](0) += costs_all(i);
            control_costs[d](num_vars_free_-1) += costs_all(num_vars_all_-(i+1));
        }
    }


    return true;
}

bool CovariantTrajectoryPolicy::updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights)
{
    ROS_ASSERT(int(updates.size()) == num_dimensions_);

    // this takes only the diagonal elements
//    for (int d=0; d<num_dimensions_; ++d)
//    {
//        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += updates[d].diagonal();
//    }

    // this averages all the updates
//    double divisor = 1.0 / num_vars_free_;
//    for (int d=0; d<num_dimensions_; ++d)
//    {
//        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_).transpose() +=
//                divisor * updates[d].colwise().sum();
//    }

    // this weights the update for each time step from the time_step_weights parameter
    for (int d=0; d<num_dimensions_; ++d)
    {
      double weight = 0.0;
      double weight_sum = 0.0;

      Eigen::VectorXd update = Eigen::VectorXd::Zero(num_vars_free_);
      for (int t=0; t<num_time_steps_; ++t)
      {
          weight = time_step_weights[d][t];
          weight_sum += weight;
          update.transpose() += updates[d].row(t) * weight;
          //ROS_INFO_STREAM("Update at time " << t << " = " << updates[d].row(t));
      }
      parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += (1.0/weight_sum)*update;
    }

    // this weights updates by number of time-steps remaining:
//    for (int d=0; d<num_dimensions_; ++d)
//    {
//        double weight=0.0;
//        double weight_sum=0.0;
//        Eigen::VectorXd update = Eigen::VectorXd::Zero(num_vars_free_);
//        for (int t=0; t<num_time_steps_; ++t)
//        {
//            weight = double(num_time_steps_ - t);
//            weight_sum += weight;
//            update.transpose() += updates[d].row(t) * weight;
//            //ROS_INFO_STREAM("Update at time " << t << " = " << updates[d].row(t));
//        }
//        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += (1.0/weight_sum)*update;
//    }

    return true;
}

bool CovariantTrajectoryPolicy::readFromFile(const std::string& abs_file_name)
{
  policy_msgs::CovariantTrajectoryPolicy ctp_msg;

  if (!usc_utilities::FileIO<policy_msgs::CovariantTrajectoryPolicy>::readFromBagFile(ctp_msg, TOPIC_NAME, abs_file_name))
    return false;

  return readFromMessage(ctp_msg);
}


bool CovariantTrajectoryPolicy::writeToFile(const std::string& abs_file_name)
{
  policy_msgs::CovariantTrajectoryPolicy ctp_msg;

  if (!writeToMessage(ctp_msg))
    return false;

  return usc_utilities::FileIO<policy_msgs::CovariantTrajectoryPolicy>::writeToBagFile(ctp_msg, TOPIC_NAME, abs_file_name);
}

//bool CovariantTrajectoryPolicy::writeToFile(const std::string& abs_file_name)
//{
//    FILE *f;
//    f = fopen(abs_file_name.c_str(), "w");
//    if (!f)
//        return false;
//
//    for (int i=free_vars_start_index_-1; i<=free_vars_end_index_+1; ++i)
//    {
//        for (int d=0; d<num_dimensions_; ++d)
//        {
//            fprintf(f,"%f\t", parameters_all_[d](i));
//        }
//        fprintf(f,"\n");
//    }
//
//    fclose(f);
//    return true;
//}

void CovariantTrajectoryPolicy::setDimensionNames(const std::vector<std::string>& dimension_names)
{
  ROS_ASSERT(int(dimension_names.size()) == num_dimensions_);
  dimension_names_ = dimension_names;
}

bool CovariantTrajectoryPolicy::writeToMessage(policy_msgs::CovariantTrajectoryPolicy& ctp_msg)
{
  ctp_msg.discretization_interval = movement_dt_;
  ctp_msg.dimension_names = dimension_names_;
  ctp_msg.points.resize(num_time_steps_+2); // two extra for start and goal
  for (int i=free_vars_start_index_-1; i<=free_vars_end_index_+1; ++i)
  {
    int ind = i - (free_vars_start_index_-1);
    ctp_msg.points[ind].positions.resize(num_dimensions_);
    for (int j=0; j<num_dimensions_; ++j)
      ctp_msg.points[ind].positions[j] = parameters_all_[j](i);
  }
  ctp_msg.nominal_start_pose = nominal_start_pose_;
  ctp_msg.derivative_costs = derivative_costs_;
  ctp_msg.cost_ridge_factor = cost_ridge_factor_;
  return true;
}

bool CovariantTrajectoryPolicy::readFromMessage(const policy_msgs::CovariantTrajectoryPolicy& ctp_msg)
{
  // first sanity check:
  if (ctp_msg.derivative_costs.size()!=3)
  {
    ROS_ERROR("CovariantTrajectoryPolicy: derivative_costs must have exactly 3 elements");
    return false;
  }
  for (unsigned int i=0; i<ctp_msg.points.size(); ++i)
  {
    if (ctp_msg.dimension_names.size() != ctp_msg.points[i].positions.size())
    {
      ROS_ERROR("CovariantTrajectoryPolicy: dimension names must have same size as positions");
      return false;
    }
  }

  num_time_steps_ = ctp_msg.points.size() - 2;
  num_dimensions_ = ctp_msg.dimension_names.size();
  dimension_names_ = ctp_msg.dimension_names;
  movement_dt_ = ctp_msg.discretization_interval;
  movement_duration_ = movement_dt_ * (num_time_steps_ + 1);
  derivative_costs_ = ctp_msg.derivative_costs;
  cost_ridge_factor_ = ctp_msg.cost_ridge_factor;
  nominal_start_pose_ = ctp_msg.nominal_start_pose;

  initializeVariables();
  initializeCosts();
  initializeBasisFunctions();

  // set the full trajectory
  for (int i=free_vars_start_index_-1; i<=free_vars_end_index_+1; ++i)
  {
    int ind = i - (free_vars_start_index_-1);
    for (int j=0; j<num_dimensions_; ++j)
      parameters_all_[j](i) = ctp_msg.points[ind].positions[j];
  }
  // duplicate start and goal:
  for (int i=0; i<free_vars_start_index_-1; ++i)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      parameters_all_[d](i) = parameters_all_[d](free_vars_start_index_-1);
    }
  }
  for (int i=free_vars_end_index_+2; i<num_vars_all_; ++i)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      parameters_all_[d](i) = parameters_all_[d](free_vars_end_index_+1);
    }
  }

  computeLinearControlCosts();

  return true;
}

bool CovariantTrajectoryPolicy::getCartesianIndices(const char** names, int size, std::vector<int>& cart_indices)
{
  cart_indices.resize(size);
  for (int i=0; i<size; ++i)
  {
    bool found=false;
    for (int d=0; d<num_dimensions_; ++d)
    {
      if (names[i] == dimension_names_[d])
      {
        found = true;
        cart_indices[i] = d;
        break;
      }
    }
    if (!found)
    {
      ROS_ERROR("Cartesian dimension %s was not found.", names[i]);
      return false;
    }
  }
  return true;
}

bool CovariantTrajectoryPolicy::getCartesianPoseIndices(std::vector<int>& cart_indices)
{
  return getCartesianIndices(CART_POSE_NAMES, 7, cart_indices);
}

bool CovariantTrajectoryPolicy::getCartesianWrenchIndices(std::vector<int>& cart_indices)
{
  return getCartesianIndices(CART_WRENCH_NAMES, 6, cart_indices);
}

bool CovariantTrajectoryPolicy::getPoseTrajectory(std::vector<geometry_msgs::Pose>& pose_trajectory)
{
  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianPoseIndices(cart_indices))
    return false;

  pose_trajectory.resize(num_vars_free_);
  for (int i=0; i<num_vars_free_; ++i)
    getPose(free_vars_start_index_+i, cart_indices, pose_trajectory[i]);
  return true;
}

bool CovariantTrajectoryPolicy::getWrenchTrajectory(std::vector<geometry_msgs::Wrench>& wrench_trajectory)
{
  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianWrenchIndices(cart_indices))
    return false;

  wrench_trajectory.resize(num_vars_free_);
  for (int i=0; i<num_vars_free_; ++i)
    getWrench(free_vars_start_index_+i, cart_indices, wrench_trajectory[i]);
  return true;
}

bool CovariantTrajectoryPolicy::getStartPose(geometry_msgs::Pose& pose)
{
  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianPoseIndices(cart_indices))
    return false;

  getPose(0, cart_indices, pose);
  return true;
}

void CovariantTrajectoryPolicy::getPose(int index, const std::vector<int>& cart_indices, geometry_msgs::Pose& pose)
{
  pose.position.x = parameters_all_[cart_indices[0]][index];
  pose.position.y = parameters_all_[cart_indices[1]][index];
  pose.position.z = parameters_all_[cart_indices[2]][index];
  pose.orientation.w = parameters_all_[cart_indices[3]][index];
  pose.orientation.x = parameters_all_[cart_indices[4]][index];
  pose.orientation.y = parameters_all_[cart_indices[5]][index];
  pose.orientation.z = parameters_all_[cart_indices[6]][index];
}

void CovariantTrajectoryPolicy::getWrench(int index, const std::vector<int>& cart_indices, geometry_msgs::Wrench& wrench)
{
  wrench.force.x = parameters_all_[cart_indices[0]][index];
  wrench.force.y = parameters_all_[cart_indices[1]][index];
  wrench.force.z = parameters_all_[cart_indices[2]][index];
  wrench.torque.x = parameters_all_[cart_indices[3]][index];
  wrench.torque.y = parameters_all_[cart_indices[4]][index];
  wrench.torque.z = parameters_all_[cart_indices[5]][index];
}

bool CovariantTrajectoryPolicy::fixQuaternionSigns()
{
  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianPoseIndices(cart_indices))
    return false;

  for (int i=1; i<num_vars_all_; ++i)
  {
    geometry_msgs::Pose pose1, pose2;
    getPose(i-1, cart_indices, pose1);
    getPose(i, cart_indices, pose2);
    double dot = pose1.orientation.w * pose2.orientation.w +
        pose1.orientation.x * pose2.orientation.x +
        pose1.orientation.y * pose2.orientation.y +
        pose1.orientation.z * pose2.orientation.z;
    if (dot < 0)
    {
      parameters_all_[cart_indices[3]][i] = -pose2.orientation.w;
      parameters_all_[cart_indices[4]][i] = -pose2.orientation.x;
      parameters_all_[cart_indices[5]][i] = -pose2.orientation.y;
      parameters_all_[cart_indices[6]][i] = -pose2.orientation.z;
    }
  }

	return true;
}

bool CovariantTrajectoryPolicy::transformCartesianPosePolicy(geometry_msgs::Pose& pose)
{
  tf::Transform transform_pose;
  tf::poseMsgToTF(pose, transform_pose);

  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianPoseIndices(cart_indices))
    return false;

  for (int i=0; i<num_vars_all_; ++i)
  {
    geometry_msgs::Pose pose;
    getPose(i, cart_indices, pose);

    tf::Transform cur_pose;
    tf::poseMsgToTF(pose, cur_pose);
    cur_pose = transform_pose * cur_pose;
    tf::poseTFToMsg(cur_pose, pose);

    parameters_all_[cart_indices[0]][i] = pose.position.x;
    parameters_all_[cart_indices[1]][i] = pose.position.y;
    parameters_all_[cart_indices[2]][i] = pose.position.z;
    parameters_all_[cart_indices[3]][i] = pose.orientation.w;
    parameters_all_[cart_indices[4]][i] = pose.orientation.x;
    parameters_all_[cart_indices[5]][i] = pose.orientation.y;
    parameters_all_[cart_indices[6]][i] = pose.orientation.z;

  }

  return true;
}

bool CovariantTrajectoryPolicy::transformCartesianWrenchPolicy(geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose pose_only_orient = pose;
  pose_only_orient.position.x = 0.0;
  pose_only_orient.position.y = 0.0;
  pose_only_orient.position.z = 0.0;
  tf::Transform transform_pose;
  tf::poseMsgToTF(pose_only_orient, transform_pose);

  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianWrenchIndices(cart_indices))
    return false;

  for (int i=0; i<num_vars_all_; ++i)
  {
    geometry_msgs::Wrench wrench;
    getWrench(i, cart_indices, wrench);

    tf::Vector3 force(wrench.force.x, wrench.force.y, wrench.force.z);
    force = transform_pose * force;
    tf::Vector3 torque(wrench.torque.x, wrench.torque.y, wrench.torque.z);
    torque = transform_pose * torque;

    parameters_all_[cart_indices[0]][i] = force.x();
    parameters_all_[cart_indices[1]][i] = force.y();
    parameters_all_[cart_indices[2]][i] = force.z();
    parameters_all_[cart_indices[3]][i] = torque.x();
    parameters_all_[cart_indices[4]][i] = torque.y();
    parameters_all_[cart_indices[5]][i] = torque.z();
  }

  return true;
}

bool CovariantTrajectoryPolicy::setWrenchTrajectory(const std::vector<geometry_msgs::WrenchStamped>& wrench_trajectory)
{
  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianWrenchIndices(cart_indices))
    return false;

  for (int i=0; i<num_vars_free_; ++i)
  {
    int index = free_vars_start_index_ + i;
    const geometry_msgs::Wrench& wrench = wrench_trajectory[i].wrench;
    parameters_all_[cart_indices[0]][index] = wrench.force.x;
    parameters_all_[cart_indices[1]][index] = wrench.force.y;
    parameters_all_[cart_indices[2]][index] = wrench.force.z;
    parameters_all_[cart_indices[3]][index] = wrench.torque.x;
    parameters_all_[cart_indices[4]][index] = wrench.torque.y;
    parameters_all_[cart_indices[5]][index] = wrench.torque.z;
  }
  return true;
}

bool CovariantTrajectoryPolicy::setPoseTrajectory(const std::vector<geometry_msgs::Pose>& pose_trajectory)
{
  // find the cartesian indices:
  std::vector<int> cart_indices;
  if (!getCartesianPoseIndices(cart_indices))
    return false;

  for (int i=0; i<num_vars_free_; ++i)
  {
    const geometry_msgs::Pose& pose = pose_trajectory[i];
    int index = i + free_vars_start_index_;

    parameters_all_[cart_indices[0]][index] = pose.position.x;
    parameters_all_[cart_indices[1]][index] = pose.position.y;
    parameters_all_[cart_indices[2]][index] = pose.position.z;
    parameters_all_[cart_indices[3]][index] = pose.orientation.w;
    parameters_all_[cart_indices[4]][index] = pose.orientation.x;
    parameters_all_[cart_indices[5]][index] = pose.orientation.y;
    parameters_all_[cart_indices[6]][index] = pose.orientation.z;

  }
  return true;
}

int CovariantTrajectoryPolicy::findDimensionByName(const std::string& dimension_name)
{
  for (unsigned int i=0; i<dimension_names_.size(); ++i)
  {
    if (dimension_names_[i] == dimension_name)
      return i;
  }
  return -1;
}

}
