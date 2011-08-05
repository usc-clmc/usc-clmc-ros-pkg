/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   dmp_dual_ik_controller.cpp

  \author Peter Pastor, Alexander Herzog
  \date   Jan 12, 2011

 *********************************************************************/

// system includes
#include <boost/thread.hpp>
#include <sstream>

// ros includes
#include <ros/callback_queue.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/constants.h>

#include <robot_info/robot_info.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/dmp_dual_ik_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_controller_implementation.h>

// import most common Eigen types
using namespace Eigen;

PLUGINLIB_DECLARE_CLASS(pr2_dynamic_movement_primitive_controller,
                        DMPDualIkController, pr2_dynamic_movement_primitive_controller::DMPDualIkController, pr2_controller_interface::Controller)

namespace pr2_dynamic_movement_primitive_controller
{

DMPDualIkController::DMPDualIkController() :
  initialized_(false), publishing_rate_(15), publishing_counter_(0), publisher_buffer_size_(0), visualization_line_counter_(0), visualization_line_rate_(5),
      visualization_line_max_points_(100), visualization_line_points_index_(0), keep_restposture_fixed_for_testing_(false), last_frame_set_(false), num_joints_(0)
{
  robot_info::RobotInfo::initialize();
}

bool DMPDualIkController::init(pr2_mechanism_model::RobotState* robot_state,
                           ros::NodeHandle& node_handle)
{
  node_handle_ = node_handle;

  ROS_VERIFY(readParameters());
  ROS_VERIFY(initRTPublisher());

  std::vector<std::string> robot_parts;
  ROS_VERIFY(usc_utilities::read(node_handle_, "robot_parts", robot_parts));

  std::vector<std::string> joint_names;
  ROS_VERIFY(robot_info::RobotInfo::getArmJointNames(robot_parts, joint_names));

  num_joints_ = (int)joint_names.size();
  ROS_INFO("Initializing DMP IK controller with >%i< joints.", num_joints_);
  int num_dof = usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + num_joints_;

  desired_positions_ = Eigen::VectorXd::Zero(num_dof);
  desired_velocities_ = Eigen::VectorXd::Zero(num_dof);
  desired_accelerations_ = Eigen::VectorXd::Zero(num_dof);

  goal_ = Eigen::VectorXd::Zero(num_dof);
  start_ = Eigen::VectorXd::Zero(num_dof);
  local_vector_ = Eigen::VectorXd::Zero(num_dof);

  // get dmp controller definition
  std::string class_name;
  ROS_VERIFY(usc_utilities::read(node_handle_, "dmp_implementation", class_name));
  if (class_name == "ICRA2009DMPControllerImplementation")
  {
    dmp_controller_.reset(new DMPControllerImplementation<dmp::ICRA2009DMP> ());
  }
  else
  {
    ROS_ERROR("Could not find implementation of controller >%s<.", class_name.c_str());
    return (initialized_ = false);
  }

  // initialize dmp controller
  std::vector<std::string> controller_variable_names;
  ROS_VERIFY(usc_utilities::read(node_handle_, "trajectory/variable_names", controller_variable_names));
  ROS_VERIFY(dmp_controller_->initialize(node_handle_.getNamespace(), controller_variable_names));

  // initialize cartesian controller
  cart_controller_.reset(new ChildController());
  if (!cart_controller_->init(robot_state, node_handle_))
  {
    ROS_ERROR("Could not initialize Cartesian controller.");
    return (initialized_ = false);
  }

//  for (int i = 0; i < usc_utilities::Constants::N_CART; i++)
//  {
//    actual_endeffector_linear_twist_(i) = 0.0;
//    desired_endeffector_linear_twist_(i) = 0.0;
//  }

  ROS_DEBUG("Done initializing DMP IK controller.");
  return (initialized_ = true);
}

bool DMPDualIkController::getArmRelatedVariables(const std::string& handle_namespace,
                                             std::string& controller_handle_namespace)
{
  // TODO: change this
  if (handle_namespace.compare(0, 6, std::string("/l_arm")) == 0)
  {
    controller_handle_namespace.assign("/l_arm_dmp_ik_controller");
  }
  else if (handle_namespace.compare(0, 6, std::string("/r_arm")) == 0)
  {
    controller_handle_namespace.assign("/r_arm_dmp_ik_controller");
  }
  else
  {
    ROS_ERROR("Invalid namespace: %s", handle_namespace.c_str());
    return false;
  }
  return true;
}

bool DMPDualIkController::readParameters()
{
  std::string controller_handle_namespace;
  ROS_VERIFY(getArmRelatedVariables(node_handle_.getNamespace(), controller_handle_namespace));
  ros::NodeHandle controller_handle(controller_handle_namespace);
  ROS_VERIFY(usc_utilities::read(controller_handle, std::string("root_name"), root_name_));
  ROS_VERIFY(usc_utilities::read(controller_handle, std::string("keep_restposture_fixed_for_testing"), keep_restposture_fixed_for_testing_));
  ROS_VERIFY(usc_utilities::read(node_handle_, std::string("publisher_buffer_size"), publisher_buffer_size_));
  return true;
}

bool DMPDualIkController::initRTPublisher()
{
  rosrt::init();
  visualization_msgs::Marker visualization_pose_marker_actual;
  viz_marker_actual_arrow_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise<visualization_msgs::Marker> (std::string("dmp_ik_controller_marker"), 1), publisher_buffer_size_, visualization_pose_marker_actual));

  visualization_msgs::Marker visualization_pose_marker_desired;
  viz_marker_desired_arrow_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise<visualization_msgs::Marker> (std::string("dmp_ik_controller_marker"), 1), publisher_buffer_size_, visualization_pose_marker_desired));

  visualization_msgs::Marker visualization_line_marker_actual;
  visualization_line_marker_actual.points.resize(visualization_line_max_points_);
  viz_marker_actual_line_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise<visualization_msgs::Marker> (std::string("dmp_ik_controller_marker"), 1), publisher_buffer_size_, visualization_line_marker_actual));

  visualization_msgs::Marker visualization_line_marker_desired;
  visualization_line_marker_desired.points.resize(visualization_line_max_points_);
  viz_marker_desired_line_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise<visualization_msgs::Marker> (std::string("dmp_ik_controller_marker"), 1), publisher_buffer_size_, visualization_line_marker_desired));

  geometry_msgs::PoseStamped pose_stamped_actual_msg;
  pose_actual_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise<geometry_msgs::PoseStamped> (std::string("dmp_pose_actual"), 1), publisher_buffer_size_, pose_stamped_actual_msg));

  geometry_msgs::PoseStamped pose_stamped_desired_msg;
  pose_desired_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise<geometry_msgs::PoseStamped> (std::string("dmp_pose_desired"), 1), publisher_buffer_size_, pose_stamped_desired_msg));
  return true;
}

bool DMPDualIkController::initXml(pr2_mechanism_model::RobotState* robot, TiXmlElement* config)
{
  ros::NodeHandle node_handle(config->Attribute("name"));
  return init(robot, node_handle);
}

// REAL-TIME REQUIREMENTS
void DMPDualIkController::starting()
{
  cart_controller_->starting();
  execution_error_ = false;
  holdPositions();
}

// REAL-TIME REQUIREMENTS
void DMPDualIkController::update()
{
  if(execution_error_)
    return;

  if (dmp_controller_->newDMPReady())
  {
    dmp_lib::DMPPtr dmp;
    if(!dmp_controller_->getDMP(dmp))
    {
      ROS_ERROR("Could not get DMP. This should never happen (Real-time violation).");
      execution_error_ = true;
      return;
    }

    if(!dmp->getGoal(goal_))
    {
      ROS_ERROR("Could not get DMP. This should never happen (Real-time violation).");
      execution_error_ = true;
      return;
    }

    getDesiredPosition();
    if(!adjustVariables(desired_positions_, start_))
    {
      ROS_ERROR("Could not rearange DMP variables (Real-time violation).");
      execution_error_ = true;
      return;
    }

    if(!dmp->changeStart(start_))
    {
      ROS_ERROR("Could not get start of the DMP (Real-time violation).");
      execution_error_ = true;
      return;
    }
  }

  // integrate DMP
  if (dmp_controller_->isRunning(desired_positions_, desired_velocities_, desired_accelerations_))
  {
    if (!setDesiredState())
    {
      ROS_ERROR("Could not set desired (Real-time violation).");
      execution_error_ = true;
      return;
    }
  }
  else
  {

    if (!holdPositions())
    {
      ROS_ERROR("Failed ot hold positions (Real-time violation).");
      execution_error_ = true;
      return;
    }
  }

  visualize();
  cart_controller_->update();
}

// REAL-TIME REQUIREMENTS
void DMPDualIkController::stopping()
{
  cart_controller_->stopping();
}

// REAL-TIME REQUIREMENTS
bool DMPDualIkController::setDesiredState()
{

  int num_used_variables;
  double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
  int num_quat_set = 0;
  if (dmp_controller_->getNumUsedVariables(num_used_variables))
  {
    // set only those variables that are used
    for (int i = 0; i < num_used_variables; ++i)
    {
      int index;
      if (!dmp_controller_->getVariableNameMap().getSupportedVariableIndex(i, index))
      {
        ROS_ERROR("Could not get index >%i<.", i);
        return false;
      }
      index++;
      int local_index = index - 1;

      //ROS_INFO("Getting index >%i< (index = >%i<) (local_index = >%i<)", i, index, local_index);

      // set desired positions and linear velocities
      if ((local_index >= 0) && (local_index < usc_utilities::Constants::N_CART))
      {
        cart_controller_->kdl_pose_desired_.p(local_index) = desired_positions_(local_index);
        if (local_index == usc_utilities::Constants::X)
        {
          // ROS_INFO("Setting cart des STATE X velocity to %f (local_index = >%i<) (index = >%i<)", desired_velocities_(local_index), local_index, index);
          cart_controller_->kdl_twist_desired_.vel.x(desired_velocities_(local_index));
        }
        else if (local_index == usc_utilities::Constants::Y)
        {
          // ROS_INFO("Setting cart des STATE Y velocity to %f (local_index = >%i<) (index = >%i<)", desired_velocities_(local_index), local_index, index);
          cart_controller_->kdl_twist_desired_.vel.y(desired_velocities_(local_index));
        }
        else if (local_index == usc_utilities::Constants::Z)
        {
          // ROS_INFO("Setting cart des STATE Z velocity to %f (local_index = >%i<) (index = >%i<)", desired_velocities_(local_index), local_index, index);
          cart_controller_->kdl_twist_desired_.vel.z(desired_velocities_(local_index));
        }
      }

      else if ((local_index >= usc_utilities::Constants::N_CART) && (local_index < usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT))
      {
        // set desired orientation and angular velocities
        if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QW)
        {
          num_quat_set++;
          qw = desired_positions_(local_index);
        }
        else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QX)
        {
          num_quat_set++;
          qx = desired_positions_(local_index);
          //ROS_INFO("Setting cart des ORIENT X velocity to %f (local_index = >%i<) (index = >%i<)", desired_velocities_(local_index), local_index, index);
          cart_controller_->kdl_twist_desired_.rot.x(desired_velocities_(local_index));
        }
        else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QY)
        {
          num_quat_set++;
          qy = desired_positions_(local_index);
          //ROS_INFO("Setting cart des ORIENT Y velocity to %f (local_index = >%i<) (index = >%i<)", desired_velocities_(local_index), local_index, index);
          cart_controller_->kdl_twist_desired_.rot.y(desired_velocities_(local_index));
        }
        else if (local_index == usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ)
        {
          num_quat_set++;
          qz = desired_positions_(local_index);
          //ROS_INFO("Setting cart des ORIENT Z velocity to %f (local_index = >%i<) (index = >%i<)", desired_velocities_(local_index), local_index, index);
          cart_controller_->kdl_twist_desired_.rot.z(desired_velocities_(local_index));
        }
      }

      // set desired nullspace posture
      else if ((local_index >= usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT) &&
          (local_index < usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + num_joints_))
      {
        int joint_index = local_index - (usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT);
        //ROS_INFO("Setting rest posture (joint_index = >%i<)", joint_index);
        cart_controller_->rest_posture_joint_configuration_(joint_index) = desired_positions_(local_index);
      }
      else
      {
        ROS_ERROR("Unknown index >%i< (Real-time violation).", index);
        return false;
      }
    }
  }
  else
  {
    // set endeffector positions, linear velocities, and linear accelerations
    for (int i = usc_utilities::Constants::X; i <= usc_utilities::Constants::Z; ++i)
    {
      cart_controller_->kdl_pose_desired_.p(i) = desired_positions_(i);
    }
    cart_controller_->kdl_twist_desired_.vel.x(desired_velocities_(usc_utilities::Constants::X));
    cart_controller_->kdl_twist_desired_.vel.y(desired_velocities_(usc_utilities::Constants::Y));
    cart_controller_->kdl_twist_desired_.vel.z(desired_velocities_(usc_utilities::Constants::Z));

    // set endeffector orientation, angular velocities, and angular accelerations
    qw = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW);
    qx = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX);
    qy = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY);
    qz = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ);
    num_quat_set = usc_utilities::Constants::N_QUAT;

    // set angular velocities
    cart_controller_->kdl_twist_desired_.rot.x(desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX));
    cart_controller_->kdl_twist_desired_.rot.y(desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY));
    cart_controller_->kdl_twist_desired_.rot.z(desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ));

    // set desired nullspace posture
    for (int i = 0; i < num_joints_; ++i)
    {
      cart_controller_->rest_posture_joint_configuration_(i) = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + i);
    }
  }

  if ((num_quat_set != 0) && (num_quat_set != usc_utilities::Constants::N_QUAT))
  {
    ROS_ERROR("Missing quaternion number. Got >%i<. Cannot set desired orientation.", num_quat_set);
    return false;
  }
  cart_controller_->kdl_pose_desired_.M = KDL::Rotation::Quaternion(qx, qy, qz, qw);


//  for (int i = usc_utilities::Constants::X; i <= usc_utilities::Constants::Z; ++i)
//  {
//    desired_endeffector_linear_twist_(i) = desired_velocities_(i);
//    actual_endeffector_linear_twist_(i) = cart_controller_->kdl_twist_measured_(i);
//  }

  return true;
}

// REAL-TIME REQUIREMENTS
bool DMPDualIkController::adjustVariables(const Eigen::VectorXd& input_vector,
                                      Eigen::VectorXd& output_vector)
{
  if (input_vector.size() != output_vector.size())
  {
    ROS_ERROR("Size of input vector >%i< must equal size of output vector >%i< (Real-time violation).", (int)input_vector.size(), (int)output_vector.size());
    return false;
  }
  local_vector_ = input_vector;

  int num_used_variables;
  if (dmp_controller_->getNumUsedVariables(num_used_variables))
  {
    // set only those variables that are used
    for (int i = 0; i < num_used_variables; ++i)
    {
      int index;
      if (!dmp_controller_->getVariableNameMap().getSupportedVariableIndex(i, index))
      {
        ROS_ERROR("Could not get index >%i< to remap variables (Real-time violation).", i);
        return false;
      }
      output_vector(i) = local_vector_(index);
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
//bool DMPDualIkController::transformCommand(Eigen::VectorXd& trajectory_point,
//                                       bool& movement_finished,
//                                       const double execution_duration,
//                                       const int num_samples)
//{
//
//    pr2_tasks_transforms::TaskTransforms::TransformType transform_type = dmp_controller_.getCurrentType();
//    int transform_type_id = static_cast<int>(transform_type);
//
//    ROS_VERIFY(dmp_controller_.getCurrentDMP()->propagateStep(dmp_pos_vel_acc_trajectory_points_[transform_type_id], movement_finished, execution_duration, num_samples));
//    ROS_VERIFY(task_frame_transformer_.getRobotTransform(transform_type, dmp_pos_vel_acc_trajectory_points_[transform_type_id], controller_trajectory_point_));
//
//    return true;
//}

// REAL-TIME REQUIREMENTS
bool DMPDualIkController::holdPositions()
{
  if (!getDesiredPosition())
  {
    return false;
  }
  for (int i = 0; i < desired_velocities_.size(); ++i)
  {
    desired_velocities_(i) = 0.0;
    desired_accelerations_(i) = 0.0;
  }
  if (!setDesiredState())
  {
    return false;
  }
  return true;
}

// REAL-TIME REQUIREMENTS
bool DMPDualIkController::getDesiredPosition()
{
  for (int i = usc_utilities::Constants::X; i <= usc_utilities::Constants::Z; ++i)
  {
    desired_positions_(i) = cart_controller_->kdl_pose_desired_.p(i);
  }
  desired_velocities_(usc_utilities::Constants::X) = cart_controller_->kdl_twist_desired_.vel.x();
  desired_velocities_(usc_utilities::Constants::Y) = cart_controller_->kdl_twist_desired_.vel.y();
  desired_velocities_(usc_utilities::Constants::Z) = cart_controller_->kdl_twist_desired_.vel.z();

  double  qw, qx, qy, qz;
  cart_controller_->kdl_pose_desired_.M.GetQuaternion(qx, qy, qz, qw);
  desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW) = qw;
  desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX) = qx;
  desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY) = qy;
  desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ) = qz;

  // set angular velocities
  desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX) = cart_controller_->kdl_twist_desired_.rot.x();
  desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY) = cart_controller_->kdl_twist_desired_.rot.y();
  desired_velocities_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ) = cart_controller_->kdl_twist_desired_.rot.z();

  // set desired nullspace posture
  for (int i = 0; i < num_joints_; ++i)
  {
    desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + i) = cart_controller_->rest_posture_joint_configuration_(i);
  }
  return true;
}

// REAL-TIME REQUIREMENTS
void DMPDualIkController::visualize()
{

  publishing_counter_++;
  if (publishing_counter_ % publishing_rate_ == 0)
  {
    publishing_counter_ = 0;

    Eigen::Matrix<double, 3, 1> velocity_vec;
    Eigen::Matrix<double, 3, 1> world_vec;
    Eigen::Quaternion<double> eigen_quat;

    visualization_msgs::MarkerPtr vis_marker_actual_arrow = viz_marker_actual_arrow_publisher_->allocate();
    if (vis_marker_actual_arrow)
    {
      vis_marker_actual_arrow->header.frame_id = std::string("/") + root_name_;
      vis_marker_actual_arrow->header.stamp = ros::Time::now();
      vis_marker_actual_arrow->ns = "DMPActualArrow";
      vis_marker_actual_arrow->type = visualization_msgs::Marker::ARROW;
      vis_marker_actual_arrow->action = visualization_msgs::Marker::ADD;
      vis_marker_actual_arrow->id = 1;
      vis_marker_actual_arrow->scale.x = 0.8 * cart_controller_->kdl_twist_measured_(0); //desired_endeffector_linear_twist_(0);
      vis_marker_actual_arrow->scale.y = 0.8 * cart_controller_->kdl_twist_measured_(1); //desired_endeffector_linear_twist_(1);
      vis_marker_actual_arrow->scale.z = 0.8 * cart_controller_->kdl_twist_measured_(2); //desired_endeffector_linear_twist_(2);
      vis_marker_actual_arrow->color.r = 0.0f;
      vis_marker_actual_arrow->color.g = 0.0f;
      vis_marker_actual_arrow->color.b = 1.0f;
      vis_marker_actual_arrow->color.a = 0.5;
      vis_marker_actual_arrow->lifetime = ros::Duration();
      vis_marker_actual_arrow->pose.position.x = cart_controller_->kdl_real_pose_measured_.p.x();
      vis_marker_actual_arrow->pose.position.y = cart_controller_->kdl_real_pose_measured_.p.y();
      vis_marker_actual_arrow->pose.position.z = cart_controller_->kdl_real_pose_measured_.p.z();
      double qx, qy, qz, qw;
      cart_controller_->kdl_real_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
      vis_marker_actual_arrow->pose.orientation.x = qx;
      vis_marker_actual_arrow->pose.orientation.y = qy;
      vis_marker_actual_arrow->pose.orientation.z = qz;
      vis_marker_actual_arrow->pose.orientation.w = qw;

      for (int i = 0; i < usc_utilities::Constants::N_CART; i++)
      {
        velocity_vec(i) = cart_controller_->kdl_twist_measured_(i); //actual_endeffector_linear_twist_(i);
        world_vec(i) = 0.0;
      }
      world_vec(0) = 1.0;
      eigen_quat.setFromTwoVectors(world_vec, velocity_vec);

      double length = velocity_vec.norm();
      if (length > 0.01)
      {
        vis_marker_actual_arrow->scale.x = length;
      }
      else
      {
        vis_marker_actual_arrow->scale.x = 0.01;
      }
      vis_marker_actual_arrow->scale.y = 0.25;
      vis_marker_actual_arrow->scale.z = 0.25;

      if ((isinf(eigen_quat.x()) == 0) && (isnan(eigen_quat.x()) == 0) && (isinf(eigen_quat.y()) == 0) && (isnan(eigen_quat.y()) == 0)
          && (isinf(eigen_quat.z()) == 0) && (isnan(eigen_quat.z()) == 0) && (isinf(eigen_quat.w()) == 0) && (isnan(eigen_quat.w()) == 0))
      {
        vis_marker_actual_arrow->pose.orientation.w = eigen_quat.w();
        vis_marker_actual_arrow->pose.orientation.x = eigen_quat.x();
        vis_marker_actual_arrow->pose.orientation.y = eigen_quat.y();
        vis_marker_actual_arrow->pose.orientation.z = eigen_quat.z();
      }
      viz_marker_actual_arrow_publisher_->publish(vis_marker_actual_arrow);
    }
    else
    {
      ROS_ERROR("skipping visualization");
    }

    visualization_msgs::MarkerPtr vis_marker_desired_arrow = viz_marker_desired_arrow_publisher_->allocate();
    if (vis_marker_desired_arrow)
    {
      vis_marker_desired_arrow->header.frame_id = std::string("/") + root_name_;
      vis_marker_desired_arrow->header.stamp = ros::Time::now();
      vis_marker_desired_arrow->ns = "DMPDesiredArrow";
      vis_marker_desired_arrow->type = visualization_msgs::Marker::ARROW;
      vis_marker_desired_arrow->action = visualization_msgs::Marker::ADD;
      vis_marker_desired_arrow->id = 2;
      vis_marker_desired_arrow->scale.x = 0.8 * desired_velocities_(0); //desired_endeffector_linear_twist_(0);
      vis_marker_desired_arrow->scale.y = 0.8 * desired_velocities_(1); //desired_endeffector_linear_twist_(1);
      vis_marker_desired_arrow->scale.z = 0.8 * desired_velocities_(2); //desired_endeffector_linear_twist_(2);
      vis_marker_desired_arrow->color.r = 0.0f;
      vis_marker_desired_arrow->color.g = 1.0f;
      vis_marker_desired_arrow->color.b = 0.0f;
      vis_marker_desired_arrow->color.a = 0.5;
      vis_marker_desired_arrow->lifetime = ros::Duration();
      vis_marker_desired_arrow->pose.position.x = desired_positions_(usc_utilities::Constants::X);
      vis_marker_desired_arrow->pose.position.y = desired_positions_(usc_utilities::Constants::Y);
      vis_marker_desired_arrow->pose.position.z = desired_positions_(usc_utilities::Constants::Z);
      vis_marker_desired_arrow->pose.orientation.w = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW);
      vis_marker_desired_arrow->pose.orientation.x = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX);
      vis_marker_desired_arrow->pose.orientation.y = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY);
      vis_marker_desired_arrow->pose.orientation.z = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ);

      for (int i = 0; i < usc_utilities::Constants::N_CART; i++)
      {
        velocity_vec(i) = desired_velocities_(i); //desired_endeffector_linear_twist_(i);
        world_vec(i) = 0.0;
      }
      world_vec(0) = 1.0;
      eigen_quat.setFromTwoVectors(world_vec, velocity_vec);

      double length = velocity_vec.norm();
      if (length > 0.01)
      {
        vis_marker_desired_arrow->scale.x = length;
      }
      else
      {
        vis_marker_desired_arrow->scale.x = 0.01;
      }
      vis_marker_desired_arrow->scale.y = 0.25;
      vis_marker_desired_arrow->scale.z = 0.25;

      if ((isinf(eigen_quat.x()) == 0) && (isnan(eigen_quat.x()) == 0) && (isinf(eigen_quat.y()) == 0) && (isnan(eigen_quat.y()) == 0)
          && (isinf(eigen_quat.z()) == 0) && (isnan(eigen_quat.z()) == 0) && (isinf(eigen_quat.w()) == 0) && (isnan(eigen_quat.w()) == 0))
      {
        vis_marker_desired_arrow->pose.orientation.x = eigen_quat.x();
        vis_marker_desired_arrow->pose.orientation.y = eigen_quat.y();
        vis_marker_desired_arrow->pose.orientation.z = eigen_quat.z();
        vis_marker_desired_arrow->pose.orientation.w = eigen_quat.w();
      }

      viz_marker_desired_arrow_publisher_->publish(vis_marker_desired_arrow);
    }

    geometry_msgs::PoseStampedPtr pose_actual = pose_actual_publisher_->allocate();
    if (pose_actual)
    {
      pose_actual->header.frame_id = std::string("/") + root_name_;
      pose_actual->header.stamp = ros::Time::now();
      pose_actual->header.seq = 0;
      pose_actual->pose.position.x = cart_controller_->kdl_real_pose_measured_.p.x();
      pose_actual->pose.position.y = cart_controller_->kdl_real_pose_measured_.p.y();
      pose_actual->pose.position.z = cart_controller_->kdl_real_pose_measured_.p.z();
      double qx, qy, qz, qw;
      cart_controller_->kdl_real_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
      pose_actual->pose.orientation.x = qx;
      pose_actual->pose.orientation.y = qy;
      pose_actual->pose.orientation.z = qz;
      pose_actual->pose.orientation.w = qw;
      pose_actual_publisher_->publish(pose_actual);
    }
    else
    {
      ROS_ERROR("skipping visualization");
    }

    geometry_msgs::PoseStampedPtr pose_desired = pose_desired_publisher_->allocate();
    if (pose_desired)
    {
      pose_desired->header.frame_id = std::string("/") + root_name_;
      pose_desired->header.stamp = ros::Time::now();
      pose_desired->header.seq = 0;
      pose_desired->pose.position.x = desired_positions_(usc_utilities::Constants::X);
      pose_desired->pose.position.y = desired_positions_(usc_utilities::Constants::Y);
      pose_desired->pose.position.z = desired_positions_(usc_utilities::Constants::Z);
      pose_desired->pose.orientation.w = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW);
      pose_desired->pose.orientation.x = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX);
      pose_desired->pose.orientation.y = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY);
      pose_desired->pose.orientation.z = desired_positions_(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ);
      pose_desired_publisher_->publish(pose_desired);
    }
    else
    {
      ROS_ERROR("skipping visualization");
    }

    visualization_line_counter_++;
    if (visualization_line_counter_ % visualization_line_rate_ == 0)
    {
      visualization_line_counter_ = 0;

      visualization_msgs::MarkerPtr vis_marker_actual_line = viz_marker_actual_line_publisher_->allocate();
      if (vis_marker_actual_line)
      {
        vis_marker_actual_line->header.frame_id = std::string("/") + root_name_;
        vis_marker_actual_line->header.stamp = ros::Time::now();
        vis_marker_actual_line->header.seq = 0;
        vis_marker_actual_line->ns = "DMPActualLine";
        vis_marker_actual_line->type = visualization_msgs::Marker::LINE_STRIP;
        vis_marker_actual_line->id = 3;
        vis_marker_actual_line->scale.x = 0.006;
        vis_marker_actual_line->scale.y = 0.006;
        vis_marker_actual_line->scale.z = 0.006;
        // vis_marker_actual_line->lifetime = ros::Duration();
        vis_marker_actual_line->color.r = 0.0f;
        vis_marker_actual_line->color.g = 0.0f;
        vis_marker_actual_line->color.b = 1.0f;
        vis_marker_actual_line->color.a = 0.4;
        geometry_msgs::Point point;
        point.x = cart_controller_->kdl_real_pose_measured_.p.x();
        point.y = cart_controller_->kdl_real_pose_measured_.p.y();
        point.z = cart_controller_->kdl_real_pose_measured_.p.z();
        for (int i = visualization_line_points_index_; i < visualization_line_max_points_; ++i)
        {
          vis_marker_actual_line->points[i] = point;
        }
        viz_marker_actual_line_publisher_->publish(vis_marker_actual_line);
      }
      else
      {
        ROS_ERROR("skipping visualization (viz_marker_actual_line)");
      }

      visualization_msgs::MarkerPtr vis_marker_desired_line = viz_marker_desired_line_publisher_->allocate();
      if (vis_marker_desired_line)
      {
        vis_marker_desired_line->header.frame_id = std::string("/") + root_name_;
        vis_marker_desired_line->header.stamp = ros::Time::now();
        vis_marker_desired_line->header.seq = 0;
        vis_marker_desired_line->ns = "DMPDesiredLine";
        vis_marker_desired_line->type = visualization_msgs::Marker::LINE_STRIP;
        vis_marker_desired_line->id = 4;
        vis_marker_desired_line->scale.x = 0.006;
        vis_marker_desired_line->scale.y = 0.006;
        vis_marker_desired_line->scale.z = 0.006;
        // vis_marker_desired_line->lifetime = ros::Duration();
        vis_marker_desired_line->header.seq = 0;
        vis_marker_desired_line->color.r = 0.0f;
        vis_marker_desired_line->color.g = 1.0f;
        vis_marker_desired_line->color.b = 0.0f;
        vis_marker_desired_line->color.a = 0.4;
        geometry_msgs::Point point;
        point.x = desired_positions_(usc_utilities::Constants::X);
        point.y = desired_positions_(usc_utilities::Constants::Y);
        point.z = desired_positions_(usc_utilities::Constants::Z);
        for (int i = visualization_line_points_index_; i < visualization_line_max_points_; ++i)
        {
          vis_marker_desired_line->points[i] = point;
        }
        viz_marker_desired_line_publisher_->publish(vis_marker_desired_line);
      }
      else
      {
        ROS_ERROR("skipping visualization (viz_marker_actual_line)");
      }

      visualization_line_points_index_++;
    }
  }
}

}
