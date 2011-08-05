/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   cartesian_twist_controller_ik_with_nullspace_optimization.cpp

  \author Peter Pastor
  \date   Jan 12, 2011

 *********************************************************************/

// system includes
#include <algorithm>
#include <sstream>

// ros includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/kinfam_io.hpp>

#include <angles/angles.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/cartesian_twist_controller_ik_with_nullspace_optimization.h>

// import most common Eigen types
using namespace Eigen;

namespace pr2_dynamic_movement_primitive_controller
{

static const int NUM_JOINTS = 7;
static const int NUM_CART = 6;

CartesianTwistControllerIkWithNullspaceOptimization::CartesianTwistControllerIkWithNullspaceOptimization() :
  robot_state_(NULL), jnt_to_twist_solver_(NULL), jnt_to_pose_solver_(NULL), jnt_to_jac_solver_(NULL), num_joints_(0), publisher_counter_(0),
      publisher_buffer_size_(0), header_sequence_number_(0)
{
}

bool CartesianTwistControllerIkWithNullspaceOptimization::init(pr2_mechanism_model::RobotState *robot_state,
                                                               ros::NodeHandle& node_handle)
{

  ROS_ASSERT(robot_state);
  robot_state_ = robot_state;
  node_handle_ = node_handle;

  rest_posture_joint_configuration_ = VectorXd::Zero(NUM_JOINTS, 1);

  eigen_desired_cartesian_velocities_ = VectorXd::Zero(NUM_CART, 1);
  eigen_desired_joint_positions_ = VectorXd::Zero(NUM_JOINTS, 1);
  eigen_desired_joint_velocities_ = VectorXd::Zero(NUM_JOINTS, 1);

  eigen_chain_jacobian_ = MatrixXd::Zero(NUM_CART, NUM_JOINTS);

  eigen_jac_times_jac_transpose_ = MatrixXd::Zero(NUM_CART, NUM_CART);
  eigen_jjt_inverse_ = MatrixXd::Zero(NUM_CART, NUM_CART);
  eigen_jac_pseudo_inverse_ = MatrixXd::Zero(NUM_CART, NUM_JOINTS);
  eigen_identity_ = MatrixXd::Zero(NUM_JOINTS, NUM_JOINTS);
  eigen_identity_.setIdentity(NUM_JOINTS, NUM_JOINTS);

  eigen_desired_cartesian_velocities_.setZero(NUM_CART, 1);
  eigen_desired_joint_velocities_.setZero(NUM_JOINTS, 1);

  eigen_nullspace_projector_ = MatrixXd::Zero(NUM_JOINTS, NUM_JOINTS);
  eigen_nullspace_term_ = VectorXd::Zero(NUM_JOINTS, 1);
  eigen_nullspace_error_ = VectorXd::Zero(NUM_JOINTS, 1);

  ROS_VERIFY(readParameters());

  ROS_VERIFY(initMechanismChain());

  ROS_VERIFY(initJointPositionController(node_handle, robot_state, joint_position_controllers_));
  num_joints_ = static_cast<int> (joint_position_controllers_.size());

  ROS_VERIFY(initCartesianPidControllers());
  ROS_VERIFY(initNullspacePidControllers());

  pose_filtered_data_.resize(NUM_CART);
  pose_unfiltered_data_.resize(NUM_CART);
  if (!((filters::MultiChannelFilterBase<double>&)pose_filter_).configure(NUM_CART, node_handle_.getNamespace() + std::string("/pose_filter"), node_handle_))
  {
    ROS_ERROR("Could not create velocity filter.");
    return false;
  }

  ROS_VERIFY(initRTPublisher());

  return true;
}

bool CartesianTwistControllerIkWithNullspaceOptimization::readParameters()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, std::string("damping"), damping_));
  ROS_VERIFY(usc_utilities::read(node_handle_, std::string("publisher_buffer_size"), publisher_buffer_size_));
  ROS_VERIFY(usc_utilities::read(node_handle_, std::string("publisher_rate"), publisher_rate_));

  ros::NodeHandle cartesian_ff_gains_handle(std::string("/cartesian_pose_twist_gains"));
  ROS_VERIFY(usc_utilities::read(cartesian_ff_gains_handle, std::string("ff_trans"), ff_trans_));
  ROS_VERIFY(usc_utilities::read(cartesian_ff_gains_handle, std::string("ff_rot"), ff_rot_));
  return true;
}

bool CartesianTwistControllerIkWithNullspaceOptimization::initMechanismChain()
{
  // get name of root and tip from the parameter server as well as damping and threshold
  std::string root_name, tip_name;
  if (!node_handle_.getParam("root_name", root_name))
  {
    ROS_ERROR("Could not retrive parameter >>root_name<< from param server in the namespace %s.", node_handle_.getNamespace().c_str());
    return false;
  }
  if (!node_handle_.getParam("tip_name", tip_name))
  {
    ROS_ERROR("Could not retrive parameter >>tip_name<< from param server in the namespace %s.", node_handle_.getNamespace().c_str());
    return false;
  }
  // create robot chain from root to tip
  if (!mechanism_chain_.init(robot_state_, root_name, tip_name))
  {
    return false;
  }
  mechanism_chain_.toKDL(kdl_chain_);

  if (static_cast<int> (kdl_chain_.getNrOfJoints()) != NUM_JOINTS)
  {
    ROS_ERROR("For now, the KDL chain needs to have >%i< arm joints, but only has >%i<.", NUM_JOINTS, (int)kdl_chain_.getNrOfJoints());
    return false;
  }

  // check if joints are calibrated
  if (!mechanism_chain_.allCalibrated())
  {
    ROS_ERROR("Joints are not calibrated.");
    return false;
  }
  // ROS_INFO("Created kdl chain with %i joints.", kdl_chain_.getNrOfJoints());

  jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  kdl_chain_jacobian_.resize(NUM_JOINTS);

  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  kdl_current_joint_positions_.resize(NUM_JOINTS);
  kdl_current_joint_velocities_.resize(NUM_JOINTS);

  kdl_desired_joint_positions_.resize(NUM_JOINTS);

  return true;
}

bool CartesianTwistControllerIkWithNullspaceOptimization::initCartesianPidControllers()
{
  // constructs 3 identical pid controllers for the x,y and z translations
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(ros::NodeHandle(node_handle_, "fb_trans")))
  {
    ROS_ERROR("Could not construct pid controller for the x, y, and z translations.");
    return false;
  }
  for (int i = 0; i < 3; ++i)
  {
    cartesian_fb_pid_controllers_.push_back(pid_controller);
  }

  // constructs 3 identical pid controllers for the x,y and z rotations
  if (!pid_controller.init(ros::NodeHandle(node_handle_, "fb_rot")))
  {
    ROS_ERROR("Could not construct pid controller for the x, y, and z rotations.");
    return false;
  }
  for (int i = 0; i < 3; ++i)
  {
    cartesian_fb_pid_controllers_.push_back(pid_controller);
  }
  return true;
}

bool CartesianTwistControllerIkWithNullspaceOptimization::initNullspacePidControllers()
{
  std::string nullspace_controller_names;
  if (!node_handle_.getParam(std::string("nullspace_controller_names"), nullspace_controller_names))
  {
    ROS_ERROR("Could not retrive parameter >>nullspace_controller_names<< from param server in the namespace %s.", node_handle_.getNamespace().c_str());
    return false;
  }
  // split the names based on whitespace
  std::stringstream ss_nullspace_controller_names(nullspace_controller_names);
  std::string nullspace_controller_name;
  while (ss_nullspace_controller_names >> nullspace_controller_name)
  {
    control_toolbox::Pid pid_controller;
    if (!pid_controller.init(ros::NodeHandle(node_handle_, nullspace_controller_name)))
    {
      ROS_ERROR("Could not construct pid controller for %s.", nullspace_controller_name.c_str());
      return false;
    }
    nullspace_fb_pid_controllers_.push_back(pid_controller);
  }
  if (static_cast<int> (nullspace_fb_pid_controllers_.size()) != num_joints_)
  {
    ROS_ERROR("There are %i nullspace terms but there should be %i.", static_cast<int>(nullspace_fb_pid_controllers_.size()), num_joints_);
    return false;
  }
  return true;
}

bool CartesianTwistControllerIkWithNullspaceOptimization::initRTPublisher()
{
  rosrt::init();
  pr2_dynamic_movement_primitive_controller::PoseTwistStamped pose_twist_stamped_desired_msg;
  pose_twist_desired_publisher_.reset(
      new rosrt::Publisher<pr2_dynamic_movement_primitive_controller::PoseTwistStamped>(node_handle_.advertise<
                                                                                        pr2_dynamic_movement_primitive_controller::PoseTwistStamped> (std::string("pose_twist_desired"), 1), publisher_buffer_size_,
                                                                                        pose_twist_stamped_desired_msg));

  pr2_dynamic_movement_primitive_controller::PoseTwistStamped pose_twist_stamped_actual_msg;
  pose_twist_actual_publisher_.reset(
      new rosrt::Publisher<pr2_dynamic_movement_primitive_controller::PoseTwistStamped>(node_handle_.advertise<pr2_dynamic_movement_primitive_controller::PoseTwistStamped> (std::string("pose_twist_actual"), 1),
                                                                                        publisher_buffer_size_,
                                                                                        pose_twist_stamped_actual_msg));

  pr2_dynamic_movement_primitive_controller::NullspaceTermStamped nullspace_term_msg;
  nullspace_term_publisher_.reset(
      new rosrt::Publisher<pr2_dynamic_movement_primitive_controller::NullspaceTermStamped>(node_handle_.advertise<pr2_dynamic_movement_primitive_controller::NullspaceTermStamped> (std::string("nullspace_term"), 1), publisher_buffer_size_, nullspace_term_msg));

  //    pr2_dynamic_movement_primitive_controller::ControllerStatus controller_status_msg;
  //    controller_status_msg.actual_pose.resize(NUM_JOINTS);
  //    controller_status_msg.desired_pose.resize(NUM_JOINTS);
  //    controller_status_msg.actual_twist.resize(NUM_CART);
  //    controller_status_msg.desired_twist.resize(NUM_CART);
  //    controller_status_msg.actual_joint_positions.resize(NUM_JOINTS);
  //    controller_status_msg.desired_joint_positions.resize(NUM_JOINTS);
  //    controller_status_msg.actual_joint_velocities.resize(NUM_JOINTS);
  //    controller_status_msg.desired_joint_velocities.resize(NUM_JOINTS);
  //    controller_status_publisher_.reset(new rosrt::Publisher<pr2_dynamic_movement_primitive_controller::ControllerStatus>(node_handle_.advertise<
  //            pr2_dynamic_movement_primitive_controller::ControllerStatus> (std::string("controller_status"), 1), publisher_buffer_size_, controller_status_msg));

  return true;
}

void CartesianTwistControllerIkWithNullspaceOptimization::starting()
{

  // reset cartesian space pid controllers
  for (int i = 0; i < NUM_CART; ++i)
  {
    cartesian_fb_pid_controllers_[i].reset();
  }

  for (int i = 0; i < num_joints_; ++i)
  {
    nullspace_fb_pid_controllers_[i].reset();
  }

  // start joint velocity controller
  for (int i = 0; i < num_joints_; ++i)
  {
    joint_position_controllers_[i].starting();
  }

  // set measured twist to 0
  kdl_twist_measured_ = KDL::Twist::Zero();

  // get the joint positions and velocities
  mechanism_chain_.getPositions(kdl_current_joint_positions_);

  for (int i = 0; i < num_joints_; ++i)
  {
    if (mechanism_chain_.getJoint(i)->joint_->type == urdf::Joint::CONTINUOUS)
    {
      kdl_current_joint_positions_(i) = angles::normalize_angle(kdl_current_joint_positions_(i));
    }
    // set desired to current
    eigen_desired_joint_positions_(i) = kdl_current_joint_positions_(i);
    kdl_desired_joint_positions_(i) = eigen_desired_joint_positions_(i);
    // ROS_ERROR("joint: %s = %f", mechanism_chain_.getJoint(i)->joint_->name.c_str(), eigen_desired_joint_positions_(i));

    // set desired rest posture to current
    rest_posture_joint_configuration_(i) = kdl_current_joint_positions_(i);
  }

  // set cartesian pose to current
  jnt_to_pose_solver_->JntToCart(kdl_current_joint_positions_, kdl_pose_desired_);

  last_time_ = robot_state_->getTime();
}

void CartesianTwistControllerIkWithNullspaceOptimization::update()
{

  // get time
  ros::Time time = robot_state_->getTime();
  dt_ = time - last_time_;
  last_time_ = time;

  // get the joint positions and filter them
  mechanism_chain_.getPositions(kdl_current_joint_positions_);

  // normalize angles
  for (int i = 0; i < num_joints_; ++i)
  {
    if (mechanism_chain_.getJoint(i)->joint_->type == urdf::Joint::CONTINUOUS)
    {
      kdl_current_joint_positions_(i) = angles::normalize_angle(kdl_current_joint_positions_(i));
      eigen_desired_joint_positions_(i) = angles::normalize_angle(eigen_desired_joint_positions_(i));
    }
    kdl_desired_joint_positions_(i) = eigen_desired_joint_positions_(i);
  }

  // get the chain jacobian
  // jnt_to_jac_solver_->JntToJac(kdl_current_joint_positions_, kdl_chain_jacobian_);
  jnt_to_jac_solver_->JntToJac(kdl_desired_joint_positions_, kdl_chain_jacobian_);

  // convert to (plain) eigen for easier math
  eigen_chain_jacobian_ = kdl_chain_jacobian_.data;

  // compute the pseudo inverse
  // eigen_jac_times_jac_transpose_ = eigen_chain_jacobian_ * eigen_chain_jacobian_.transpose();
  eigen_jac_times_jac_transpose_ = eigen_chain_jacobian_ * eigen_chain_jacobian_.transpose() + MatrixXd::Identity(NUM_CART, NUM_CART) * damping_;

  eigen_jac_times_jac_transpose_.computeInverse(&eigen_jjt_inverse_);

  eigen_jac_pseudo_inverse_ = eigen_chain_jacobian_.transpose() * eigen_jjt_inverse_;

  // compute the nullspace projector
  eigen_identity_.setIdentity();
  eigen_nullspace_projector_ = eigen_identity_ - (eigen_jac_pseudo_inverse_ * eigen_chain_jacobian_);

  // get cartesian pose
  jnt_to_pose_solver_->JntToCart(kdl_current_joint_positions_, kdl_real_pose_measured_);
  jnt_to_pose_solver_->JntToCart(kdl_desired_joint_positions_, kdl_pose_measured_);

  // compute twist
  kdl_twist_error_ = -diff(kdl_pose_measured_, kdl_pose_desired_);

  // filter twist error
  for (int i = 0; i < NUM_CART; ++i)
  {
    pose_unfiltered_data_[i] = kdl_twist_error_(i);
  }
  pose_filter_.update(pose_unfiltered_data_, pose_filtered_data_);
  for (int i = 0; i < NUM_CART; ++i)
  {
    kdl_twist_error_(i) = pose_filtered_data_[i];
  }

  // compute desired cartesian velocities
  for (int i = 0; i < 3; ++i)
  {
    eigen_desired_cartesian_velocities_(i) = (kdl_twist_desired_(i) * ff_trans_) + cartesian_fb_pid_controllers_[i].updatePid(kdl_twist_error_(i), dt_);
  }
  for (int i = 3; i < NUM_CART; ++i)
  {
    eigen_desired_cartesian_velocities_(i) = (kdl_twist_desired_(i) * ff_rot_) + cartesian_fb_pid_controllers_[i].updatePid(kdl_twist_error_(i), dt_);
  }

  // compute desired joint velocities
  eigen_desired_joint_velocities_ = eigen_jac_pseudo_inverse_ * eigen_desired_cartesian_velocities_;

  double error;
  for (int i = 0; i < num_joints_; ++i)
  {
    if (mechanism_chain_.getJoint(i)->joint_->type == urdf::Joint::CONTINUOUS)
    {
      error = angles::shortest_angular_distance(kdl_current_joint_positions_(i), rest_posture_joint_configuration_(i));
    }
    else
    {
      error = rest_posture_joint_configuration_(i) - kdl_current_joint_positions_(i);
    }
    eigen_nullspace_error_(i) = nullspace_fb_pid_controllers_[i].updatePid(error, dt_);
  }
  eigen_nullspace_term_ = eigen_nullspace_projector_ * eigen_nullspace_error_;
  eigen_desired_joint_velocities_ += eigen_nullspace_term_;

  // integrate desired joint velocities to get desired joint positions
  eigen_desired_joint_positions_ = eigen_desired_joint_positions_ + eigen_desired_joint_velocities_ * dt_.toSec();

  // added by schorfi/mrinal (clip the joint limits)
  for (int i = 0; i < num_joints_; ++i)
  {
    if (mechanism_chain_.getJoint(i)->joint_->type != urdf::Joint::CONTINUOUS)
    {
      if (eigen_desired_joint_positions_(i) > mechanism_chain_.getJoint(i)->joint_->limits->upper)
      {
        eigen_desired_joint_positions_(i) = mechanism_chain_.getJoint(i)->joint_->limits->upper;
      }
      if (eigen_desired_joint_positions_(i) < mechanism_chain_.getJoint(i)->joint_->limits->lower)
      {
        eigen_desired_joint_positions_(i) = mechanism_chain_.getJoint(i)->joint_->limits->lower;
      }
    }
  }

  // set joint positions and update
  for (int i = 0; i < num_joints_; ++i)
  {
    joint_position_controllers_[i].setCommand(eigen_desired_joint_positions_(i));
  }
  for (int i = 0; i < num_joints_; ++i)
  {
    joint_position_controllers_[i].update();
  }

  // get measured twist for debugging reasons...
  KDL::FrameVel framevel_measured;
  mechanism_chain_.getVelocities(kdl_current_joint_velocities_);
  jnt_to_twist_solver_->JntToCart(kdl_current_joint_velocities_, framevel_measured);
  kdl_twist_measured_ = framevel_measured.deriv();

  publish();
}

void CartesianTwistControllerIkWithNullspaceOptimization::publish()
{

  publisher_counter_++;
  if (publisher_counter_ % publisher_rate_ == 0)
  {
    header_sequence_number_++;

    double qx, qy, qz, qw;
    pr2_dynamic_movement_primitive_controller::PoseTwistStampedPtr pose_twist_stamped_desired_msg;
    pose_twist_stamped_desired_msg = pose_twist_desired_publisher_->allocate();
    if (pose_twist_stamped_desired_msg)
    {
      pose_twist_stamped_desired_msg->header.stamp = ros::Time::now();

      pose_twist_stamped_desired_msg->pose.position.x = kdl_pose_desired_.p.x();
      pose_twist_stamped_desired_msg->pose.position.y = kdl_pose_desired_.p.y();
      pose_twist_stamped_desired_msg->pose.position.z = kdl_pose_desired_.p.z();

      kdl_pose_desired_.M.GetQuaternion(qx, qy, qz, qw);
      pose_twist_stamped_desired_msg->pose.orientation.x = qx;
      pose_twist_stamped_desired_msg->pose.orientation.y = qy;
      pose_twist_stamped_desired_msg->pose.orientation.z = qz;
      pose_twist_stamped_desired_msg->pose.orientation.w = qw;

      pose_twist_stamped_desired_msg->twist.linear.x = kdl_twist_desired_.vel.x();
      pose_twist_stamped_desired_msg->twist.linear.y = kdl_twist_desired_.vel.y();
      pose_twist_stamped_desired_msg->twist.linear.z = kdl_twist_desired_.vel.z();
      pose_twist_stamped_desired_msg->twist.angular.x = kdl_twist_desired_.rot.x();
      pose_twist_stamped_desired_msg->twist.angular.y = kdl_twist_desired_.rot.y();
      pose_twist_stamped_desired_msg->twist.angular.z = kdl_twist_desired_.rot.z();

      pose_twist_desired_publisher_->publish(pose_twist_stamped_desired_msg);
    }

    pr2_dynamic_movement_primitive_controller::PoseTwistStampedPtr pose_twist_stamped_actual_msg;
    pose_twist_stamped_actual_msg = pose_twist_actual_publisher_->allocate();
    if (pose_twist_stamped_actual_msg)
    {
      pose_twist_stamped_actual_msg->header.stamp = ros::Time::now();

      pose_twist_stamped_actual_msg->pose.position.x = kdl_pose_measured_.p.x();
      pose_twist_stamped_actual_msg->pose.position.y = kdl_pose_measured_.p.y();
      pose_twist_stamped_actual_msg->pose.position.z = kdl_pose_measured_.p.z();

      kdl_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
      pose_twist_stamped_actual_msg->pose.orientation.x = qx;
      pose_twist_stamped_actual_msg->pose.orientation.y = qy;
      pose_twist_stamped_actual_msg->pose.orientation.z = qz;
      pose_twist_stamped_actual_msg->pose.orientation.w = qw;

      // pose_twist_stamped_actual_msg->twist.linear.x = kdl_twist_measured_.vel.x();
      // pose_twist_stamped_actual_msg->twist.linear.y = kdl_twist_measured_.vel.y();
      // pose_twist_stamped_actual_msg->twist.linear.z = kdl_twist_measured_.vel.z();
      // pose_twist_stamped_actual_msg->twist.angular.x = kdl_twist_measured_.rot.x();
      // pose_twist_stamped_actual_msg->twist.angular.y = kdl_twist_measured_.rot.y();
      // pose_twist_stamped_actual_msg->twist.angular.z = kdl_twist_measured_.rot.z();

      pose_twist_stamped_actual_msg->twist.linear.x = eigen_desired_cartesian_velocities_(0);
      pose_twist_stamped_actual_msg->twist.linear.y = eigen_desired_cartesian_velocities_(1);
      pose_twist_stamped_actual_msg->twist.linear.z = eigen_desired_cartesian_velocities_(2);
      pose_twist_stamped_actual_msg->twist.angular.x = eigen_desired_cartesian_velocities_(3);
      pose_twist_stamped_actual_msg->twist.angular.y = eigen_desired_cartesian_velocities_(4);
      pose_twist_stamped_actual_msg->twist.angular.z = eigen_desired_cartesian_velocities_(5);

      pose_twist_actual_publisher_->publish(pose_twist_stamped_actual_msg);
    }

    pr2_dynamic_movement_primitive_controller::NullspaceTermStampedPtr nullspace_term_stamped_msg;
    nullspace_term_stamped_msg = nullspace_term_publisher_->allocate();
    if (nullspace_term_stamped_msg)
    {
      //            nullspace_term_stamped_msg->nullspace_term_0 = eigen_nullspace_term_(0);
      //            nullspace_term_stamped_msg->nullspace_term_1 = eigen_nullspace_term_(1);
      //            nullspace_term_stamped_msg->nullspace_term_2 = eigen_nullspace_term_(2);
      //            nullspace_term_stamped_msg->nullspace_term_3 = eigen_nullspace_term_(3);
      //            nullspace_term_stamped_msg->nullspace_term_4 = eigen_nullspace_term_(4);
      //            nullspace_term_stamped_msg->nullspace_term_5 = eigen_nullspace_term_(5);
      //            nullspace_term_stamped_msg->nullspace_term_6 = eigen_nullspace_term_(6);

      nullspace_term_stamped_msg->nullspace_term_0 = eigen_nullspace_error_(0);
      nullspace_term_stamped_msg->nullspace_term_1 = eigen_nullspace_error_(1);
      nullspace_term_stamped_msg->nullspace_term_2 = eigen_nullspace_error_(2);
      nullspace_term_stamped_msg->nullspace_term_3 = eigen_nullspace_error_(3);
      nullspace_term_stamped_msg->nullspace_term_4 = eigen_nullspace_error_(4);
      nullspace_term_stamped_msg->nullspace_term_5 = eigen_nullspace_error_(5);
      nullspace_term_stamped_msg->nullspace_term_6 = eigen_nullspace_error_(6);

      nullspace_term_publisher_->publish(nullspace_term_stamped_msg);
    }

    //        pr2_dynamic_movement_primitive_controller::ControllerStatusPtr controller_status_msg;
    //        controller_status_msg = controller_status_publisher_->allocate();
    //        if (controller_status_msg)
    //        {
    //            double cmd;
    //            for (int i = 0; i < num_joints_; ++i)
    //            {
    //                controller_status_msg->actual_joint_positions[i] = joint_position_controllers_[i].getJointPosition();
    //                joint_position_controllers_[i].getCommand(cmd);
    //                controller_status_msg->desired_joint_positions[i] = cmd;
    //
    //                controller_status_msg->actual_joint_velocities[i] = joint_position_controllers_[i].getJointVelocity();
    //                controller_status_msg->desired_joint_velocities[i] = joint_position_controllers_[i].getJointVelocityError();
    //            }
    //
    //            controller_status_msg->desired_pose[0] = kdl_pose_desired_.p.x();
    //            controller_status_msg->desired_pose[1] = kdl_pose_desired_.p.y();
    //            controller_status_msg->desired_pose[2] = kdl_pose_desired_.p.z();
    //            kdl_pose_desired_.M.GetQuaternion(qx, qy, qz, qw);
    //            controller_status_msg->desired_pose[3] = qx;
    //            controller_status_msg->desired_pose[4] = qy;
    //            controller_status_msg->desired_pose[5] = qz;
    //            controller_status_msg->desired_pose[6] = qw;
    //
    //            controller_status_msg->actual_pose[0] = kdl_real_pose_measured_.p.x();
    //            controller_status_msg->actual_pose[1] = kdl_real_pose_measured_.p.y();
    //            controller_status_msg->actual_pose[2] = kdl_real_pose_measured_.p.z();
    //            kdl_real_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
    //            controller_status_msg->actual_pose[3] = qx;
    //            controller_status_msg->actual_pose[4] = qy;
    //            controller_status_msg->actual_pose[5] = qz;
    //            controller_status_msg->actual_pose[6] = qw;
    //
    //            controller_status_msg->desired_twist[0] = kdl_twist_desired_.vel.x();
    //            controller_status_msg->desired_twist[1] = kdl_twist_desired_.vel.y();
    //            controller_status_msg->desired_twist[2] = kdl_twist_desired_.vel.z();
    //            controller_status_msg->desired_twist[3] = kdl_twist_desired_.rot.x();
    //            controller_status_msg->desired_twist[4] = kdl_twist_desired_.rot.y();
    //            controller_status_msg->desired_twist[5] = kdl_twist_desired_.rot.z();
    //
    //            for (int i = 0; i < NUM_CART; ++i)
    //            {
    //                controller_status_msg->actual_twist[i] = eigen_desired_cartesian_velocities_(i);
    //            }
    //
    //            controller_status_msg->dt = dt_.toNSec();
    //
    //            controller_status_publisher_->publish(controller_status_msg);
    //        }

  }

}

bool CartesianTwistControllerIkWithNullspaceOptimization::initJointPositionController(ros::NodeHandle node_handle,
                                                                                      pr2_mechanism_model::RobotState* robot_state,
                                                                                      std::vector<JointPositionController>& joint_position_controllers)
{
  joint_position_controllers.clear();
  std::vector<std::string> joint_names;
  ROS_VERIFY(usc_utilities::read(node_handle, "joint_names", joint_names));
  for (int i = 0; i < (int)joint_names.size(); ++i)
  {
    ros::NodeHandle joint_node_handle(node_handle, joint_names[i]);
    pr2_dynamic_movement_primitive_controller::JointPositionController joint_controller;
    if (!joint_controller.init(robot_state, joint_node_handle))
    {
      ROS_ERROR("Could not initialize joint controller for joint >%s<.", joint_names[i].c_str());
      return false;
    }
    joint_position_controllers.push_back(joint_controller);
  }

  return (static_cast<int> (joint_position_controllers.size()) == NUM_JOINTS);
}

//void CartesianTwistControllerIkWithNullspaceOptimization::computeAngularVelocityError(const double* quat1, const double* quat2, double* angular_velocity_error)
//{
//    angular_velocity_error[0] = quat1[0] * quat2[1] - quat2[0] * quat1[1] - (quat1[2] * quat2[3] - quat1[3] * quat2[2]);
//    angular_velocity_error[1] = quat1[0] * quat2[2] - quat2[0] * quat1[2] - (quat1[3] * quat2[1] - quat1[1] * quat2[3]);
//    angular_velocity_error[2] = quat1[0] * quat2[3] - quat2[0] * quat1[3] - (quat1[1] * quat2[2] - quat1[2] * quat2[1]);
//}
//void CartesianTwistControllerIkWithNullspaceOptimization::getQuaternionFromRPY(const double roll, const double pitch, const double yaw, double &qx, double &qy,
//                                                                               double &qz, double &qw)
//{
//    qw = cos(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
//    qx = sin(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) - cos(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
//    qy = cos(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0);
//    qz = cos(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0) - sin(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0);
//}

} // namespace
