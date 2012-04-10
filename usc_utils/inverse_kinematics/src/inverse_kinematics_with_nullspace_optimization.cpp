/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		inverse_kinematics_with_nullspace_optimization.cpp

  \author	Peter Pastor
  \date		Jan 24, 2011

 *********************************************************************/

// system includes
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_kdl.h>

#include <usc_utilities/param_server.h>

// local includes
#include <inverse_kinematics/inverse_kinematics_with_nullspace_optimization.h>

using namespace Eigen;
using namespace robot_info;
using namespace KDL;

namespace inverse_kinematics
{

bool InverseKinematicsWithNullspaceOptimization::initialize(ros::NodeHandle node_handle)
{
  std::string start_link;
  ROS_VERIFY(usc_utilities::read(node_handle, "base_link_name", start_link));
  std::string end_link;
  ROS_VERIFY(usc_utilities::read(node_handle, "cartesian_space_dmp/" + robot_part_name_ + "/end_link_name", end_link));
  base_link_name_ = RobotInfo::getBaseFrame();
  return initialize(start_link, end_link);
}

bool InverseKinematicsWithNullspaceOptimization::initialize(const std::string& start_link,
                                                            const std::string& end_link)
{
  ROS_INFO("Initializing inverse kinematics with nullspace optimization of robot part >%s< from link >%s< to link >%s<.",
           robot_part_name_.c_str(), start_link.c_str(), end_link.c_str());

  std::vector<std::string> names;
  ROS_VERIFY(robot_info::RobotInfo::getNames(robot_part_name_, names));
  ROS_ASSERT_MSG(!names.empty(), "Invalid robot part name >%s<. Looks like it does not contain any variable names.", robot_part_name_.c_str());
  first_link_name_.assign(names[0]);
  usc_utilities::appendLeadingSlash(first_link_name_);
  ROS_INFO("Setting first link to be >%s<.", first_link_name_.c_str());

  //  std::vector<JointInfo> joint_infos;
  //  if(!RobotInfo::getJointInfo(robot_part_name_, joint_infos))
  //  {
  //    ROS_ERROR("Could not get joint infos for robot part >%s<.", robot_part_name_.c_str());
  //    return false;
  //  }
  //  num_joints_ = joint_infos.size();

  RobotInfo::getKDLChain(start_link, end_link, chain_);

  num_joints_ = chain_.getNrOfJoints();

  start_link_name_.assign(start_link);
  usc_utilities::appendLeadingSlash(start_link_name_);
  end_link_name_.assign(end_link);
  usc_utilities::appendLeadingSlash(end_link_name_);
  usc_utilities::appendLeadingSlash(base_link_name_);

  target_pose_visualizer_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/inverse_kinematics/ik_null_debug_target", 100, true);

  eigen_desired_cartesian_velocities_ = VectorXd::Zero(6, 1);
  eigen_desired_joint_positions_ = VectorXd::Zero(num_joints_, 1);
  eigen_desired_joint_velocities_ = VectorXd::Zero(num_joints_, 1);

  eigen_jac_times_jac_transpose_ = MatrixXd::Zero(6, 6);
  eigen_jjt_inverse_ = MatrixXd::Zero(6, 6);
  eigen_jac_pseudo_inverse_ = MatrixXd::Zero(6, num_joints_);
  eigen_identity_ = MatrixXd::Zero(num_joints_, num_joints_);
  eigen_identity_.setIdentity(num_joints_, num_joints_);

  eigen_nullspace_error_ = VectorXd::Zero(num_joints_, 1);
  eigen_nullspace_term_ = VectorXd::Zero(num_joints_, 1);
  eigen_nullspace_projector_ = MatrixXd::Zero(num_joints_, num_joints_);

  jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(chain_));

  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
  kdl_chain_jacobian_.resize(num_joints_);

  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

  kdl_current_joint_positions_.resize(num_joints_);
  kdl_previous_joint_positions_.resize(num_joints_);

  return (initialized_ = true);
}

bool InverseKinematicsWithNullspaceOptimization::ik(const std::vector<geometry_msgs::PoseStamped>& poses,
                                                    const std::vector<VectorXd>& rest_postures,
                                                    const VectorXd& joint_angle_seed,
                                                    std::vector<VectorXd>& joint_angles)
{
  ROS_ASSERT(initialized_);
  joint_angles.clear();

  // const double delta_t = (double) 1.0 / pr2_controller_utilities::PR2RobotInfo::PR2_DEFAULT_SAMPLING_FREQUENCY;

  if(poses.empty())
  {
    ROS_ERROR("There are no poses, cannot compute inverse kinematics.");
    return false;
  }
  if(poses.size() != rest_postures.size())
  {
    ROS_ERROR("Number of poses >%i< must equal number of rest postures >%i<.", (int)poses.size(), (int)rest_postures.size());
    return false;
  }
  for(int i=0; i<(int)rest_postures.size(); ++i)
  {
    if(rest_postures[i].size() != num_joints_)
    {
      ROS_ERROR("Rest posture contains >%i< values and therefore is invalid.", (int)rest_postures[i].size());
      return false;
    }
  }

  if(joint_angle_seed.size() != num_joints_)
  {
    ROS_ERROR("Joint angle seed contains >%i< values and therefore is invalid.", (int)joint_angle_seed.size());
    return false;
  }
  kdl_previous_joint_positions_.data = joint_angle_seed;
  ros::Time previous_time_stamp = poses[0].header.stamp;
  ros::Time start_time_stamp = ros::Time::now();

  timer_.startTimer();
  for (int i = 1; i < (int)poses.size(); ++i)
  // for (int i = 1; i < 2; ++i)
  {
    ros::Time current_time_stamp = poses[i].header.stamp;
    double delta_t = ros::Duration(current_time_stamp - previous_time_stamp).toSec();
    previous_time_stamp = current_time_stamp;

    ROS_DEBUG("Setting delta_t to >%f<.", delta_t);

    tf::PoseMsgToKDL(poses[i].pose, kdl_desired_pose_);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = poses[i].pose;
    // pose_stamped.header.frame_id = poses[i].header.frame_id;
    pose_stamped.header.frame_id = start_link_name_;
    pose_stamped.header.stamp = start_time_stamp;
    target_pose_visualizer_.publish(pose_stamped);

    // get the chain jacobian
    jnt_to_jac_solver_->JntToJac(kdl_previous_joint_positions_, kdl_chain_jacobian_);

		// ROS_INFO_STREAM(kdl_chain_jacobian_.data);

    // convert to (plain) eigen for easier math
    eigen_chain_jacobian_ = kdl_chain_jacobian_.data;

    // compute the pseudo inverse
    double damping = 1.0;
    eigen_jac_times_jac_transpose_ = eigen_chain_jacobian_ * eigen_chain_jacobian_.transpose() + MatrixXd::Identity(6, 6) * damping;
		bool is_invertible = true;
		// TODO: port this to Eigen3
		//    eigen_jac_times_jac_transpose_.computeInverseWithCheck(&eigen_jjt_inverse_, is_invertible);

    eigen_jac_pseudo_inverse_ = eigen_chain_jacobian_.transpose() * eigen_jjt_inverse_;

    // compute the nullspace projector
    eigen_identity_.setIdentity();
    eigen_nullspace_projector_ = eigen_identity_ - (eigen_jac_pseudo_inverse_ * eigen_chain_jacobian_);

    // get cartesian pose
    jnt_to_pose_solver_->JntToCart(kdl_previous_joint_positions_, kdl_previous_pose_);

    // compute twist
    kdl_twist_desired_ = -diff(kdl_desired_pose_, kdl_previous_pose_, delta_t).RefPoint(kdl_previous_pose_.p - kdl_desired_pose_.p);

    // compute desired cartesian velocities
    for (int j = 0; j < 6; j++)
    {
      eigen_desired_cartesian_velocities_(j) = kdl_twist_desired_(j);
    }

    // compute desired joint velocities
    eigen_desired_joint_velocities_ = eigen_jac_pseudo_inverse_ * eigen_desired_cartesian_velocities_;

    double gain = 5.0;
    for (int j = 0; j < num_joints_; j++)
    {
      eigen_nullspace_error_(j) = (rest_postures[i](j) - kdl_current_joint_positions_(j)) * gain;
    }
    eigen_nullspace_term_ = eigen_nullspace_projector_ * eigen_nullspace_error_;
    eigen_desired_joint_velocities_ += eigen_nullspace_term_;

    // integrate desired joint velocities to get desired joint positions
    eigen_desired_joint_positions_ = eigen_desired_joint_positions_ + eigen_desired_joint_velocities_ * delta_t;

    // set current joint positions to desired
    kdl_current_joint_positions_.data = eigen_desired_joint_positions_;

    // update the previous state for next iteration
    kdl_previous_joint_positions_ = kdl_current_joint_positions_;

    // robot_visualizer_.publishPose(robot_part_name_, base_link_name_, kdl_current_joint_positions_);
    robot_visualizer_.publishPose(robot_part_name_, first_link_name_, kdl_current_joint_positions_);
    ros::Duration(delta_t).sleep();
  }
  ROS_INFO("IK solution took %f ms.", timer_.getElapsedTimeMilliSeconds());

  return true;
}

}

