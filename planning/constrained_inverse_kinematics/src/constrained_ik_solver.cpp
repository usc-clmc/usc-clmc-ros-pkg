/*
 * constrained_ik_solver.cpp
 *
 *  Created on: Jun 10, 2011
 *      Author: kalakris
 */

#include <boost/pointer_cast.hpp>
#include <constrained_inverse_kinematics/constrained_ik_solver.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <kdl_parser/kdl_parser.hpp>
#include <conversions/ros_to_kdl.h>
#include <conversions/kdl_to_ros.h>
#include <conversions/kdl_to_eigen.h>
#include <kdl/frames.hpp>
#include <constrained_inverse_kinematics/cost_function_input.h>
#include <usc_utilities/math.h>
#include <Eigen/Dense>

using namespace KDL;
using namespace Eigen;
using namespace conversions;

namespace constrained_inverse_kinematics
{

ConstrainedIKSolver::ConstrainedIKSolver(ros::NodeHandle node_handle, const std::string& root, const std::string& tip):
    node_handle_(node_handle)
{

  // read params
  node_handle.param("max_translation_error", max_translation_error_, 0.05);
  node_handle.param("max_orientation_error", max_orientation_error_, 0.3);
  node_handle.param("position_convergence_threshold", position_convergence_threshold_, 0.001);
  node_handle.param("orientation_convergence_threshold", orientation_convergence_threshold_, 0.001);
  node_handle.param("min_joint_update_threshold", min_joint_update_threshold_, 0.0001);
  node_handle.param("max_null_space_joint_update", max_null_space_joint_update_, 0.01);
  node_handle.param("max_iterations", max_iterations_, 100);

  chain_.reset(new Chain(root, tip));
  default_fk_solver_ = chain_->fk_solver_;
  useDefaultFKSolver();

  boost::mt19937 mt19937;
  mt19937.seed(rand());
  boost::uniform_01<> uniform_01;
  random_generator_.reset(new boost::variate_generator<boost::mt19937, boost::uniform_01<> >(mt19937, uniform_01));

}

ConstrainedIKSolver::~ConstrainedIKSolver()
{
}

void ConstrainedIKSolver::registerDebugCallback(boost::function<void (const KDL::JntArray& q)> f)
{
  debug_callback_ = f;
}

void ConstrainedIKSolver::setCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function)
{
  cost_function_ = cost_function->clone();
}

void ConstrainedIKSolver::setFKSolver(boost::shared_ptr<const FKSolver> fk_solver)
{
  fk_solver_ = fk_solver;
}

void ConstrainedIKSolver::useDefaultFKSolver()
{
  fk_solver_ = default_fk_solver_;
}

bool ConstrainedIKSolver::ikLocal(const InverseKinematicsRequest& ik_request,
           const KDL::JntArray& q_in,
           IKSolution& solution) const
{
  KinematicsInfo kinematics_info;
  KDL::Frame link_to_tool_frame;
  KDL::Frame desired_tool_frame;
  KDL::Frame tool_frame;
  KDL::Twist twist;
  boost::shared_ptr<CostFunctionInput> cost_function_input(new CostFunctionInput());
  rosPoseToKdlFrame(ik_request.link_to_tool_pose, link_to_tool_frame);
  rosPoseToKdlFrame(ik_request.desired_tool_pose, desired_tool_frame);

  VectorXd error_vector(6);
  MatrixXd world_jacobian(6, chain_->num_joints_);
  int num_jacobian_rows=0;
  MatrixXd position_jacobian(3, chain_->num_joints_);
  MatrixXd position_jacobian_constraint_frame(3, chain_->num_joints_);
  MatrixXd orientation_jacobian(3, chain_->num_joints_);
  MatrixXd JJt(6,6);
  MatrixXd Jhash(chain_->num_joints_, 6);
  MatrixXd null_space(chain_->num_joints_, chain_->num_joints_);
  VectorXd damping = VectorXd::Ones(6) * 1e-4;
  VectorXd delta_theta(chain_->num_joints_);
  VectorXd cost_function_gradient;
  bool cost_function_state_validity;
  std::vector<double> cost_function_weighted_feature_values;
  VectorXd null_space_update;
  double cost_function_value;
  bool converged = false;

  solution.cost_function_value = std::numeric_limits<double>::max();
  solution.success = false;
  solution.joint_angles = q_in;

  KDL::JntArray q_prev = q_in;
  int num_iter=0;

  boost::shared_ptr<const FKSolver> my_fk_solver = default_fk_solver_;

  // constraint handling
  KDL::Rotation position_constraint_orientation;
  KDL::Rotation position_constraint_orientation_inverse;
  Eigen::Matrix3d position_constraint_orientation_inverse_eigen;
  if (ik_request.use_position_constraint)
  {
    if (ik_request.position_constraint_shape.type != geometric_shapes_msgs::Shape::BOX)
    {
      ROS_ERROR("CIK PositionConstraint: we only handle 3-d BOX constraints currently.");
      return false;
    }
    rosQuaternionToKdlRotation(ik_request.position_constraint_orientation, position_constraint_orientation);
    position_constraint_orientation_inverse = position_constraint_orientation.Inverse();
    kdlRotationToEigenMatrix3d(position_constraint_orientation_inverse, position_constraint_orientation_inverse_eigen);
  }

  KDL::JntArray q_out = q_in;

  int position_start_row, position_end_row, orientation_start_row, orientation_end_row;
  do
  {
    if (debug_callback_)
    {
      debug_callback_(q_out);
    }
    // do FK for links
    my_fk_solver->solve(q_out, kinematics_info);

    // find tool frame
    // TODO: use link specified in msg instead of last one
    tool_frame = kinematics_info.link_frames_.back() * link_to_tool_frame;

    // find error
    twist = diff(tool_frame, desired_tool_frame);

    // get jacobians
    my_fk_solver->getPositionJacobian(kinematics_info, tool_frame.p, position_jacobian);
    my_fk_solver->getOrientationJacobian(kinematics_info, orientation_jacobian);

    num_jacobian_rows = 0;
    position_start_row = num_jacobian_rows;

    // add the position jacobian rows
    if (ik_request.use_position_constraint)
    {
      // rotate the error into constraint frame
      KDL::Vector error_constraint = position_constraint_orientation_inverse * twist.vel;

      // rotate the jacobian into constraint frame
      position_jacobian_constraint_frame = position_constraint_orientation_inverse_eigen * position_jacobian;

      // now check each dimension for constraint violations
      for (int d=0; d<3; ++d)
      {
        if (fabs(error_constraint(d)) > ik_request.position_constraint_shape.dimensions[d]/2.0)
        {
          error_vector(num_jacobian_rows) = error_constraint(d);
          world_jacobian.row(num_jacobian_rows) = position_jacobian_constraint_frame.row(d);
          ++num_jacobian_rows;
        }
      }

    }
    else
    {
      num_jacobian_rows = 3;
      world_jacobian.block(0, 0, 3, chain_->num_joints_) = position_jacobian;
      for (int d=0; d<3; ++d)
        error_vector(d) = twist.vel(d);
    }

    position_end_row = num_jacobian_rows - 1;
    orientation_start_row = num_jacobian_rows;

    // add orientation jacobian rows
    if (ik_request.use_orientation_constraint)
    {
      for (int d=0; d<3; ++d)
      {
        if (fabs(twist.rot(d)) > ik_request.orientation_constraint_angular_tolerance[d])
        {
          error_vector(num_jacobian_rows) = twist.rot(d);
          world_jacobian.row(num_jacobian_rows) = orientation_jacobian.row(d);
          ++num_jacobian_rows;
        }
      }
    }
    else
    {
      world_jacobian.block(num_jacobian_rows, 0, 3, chain_->num_joints_) = orientation_jacobian;
      for (int d=0; d<3; ++d)
        error_vector(d+num_jacobian_rows) = twist.rot(d);
      num_jacobian_rows += 3;
    }

    orientation_end_row = num_jacobian_rows - 1;

    // check for convergence, and clip the errors
    converged = true;
    for (int d=position_start_row; d<=position_end_row; ++d)
    {
      if (fabs(error_vector(d)) > position_convergence_threshold_)
        converged = false;
      error_vector(d) = usc_utilities::clipAbsoluteValue(error_vector(d), max_translation_error_);
    }
    for (int d=orientation_start_row; d<=orientation_end_row; ++d)
    {
      if (fabs(error_vector(d)) > orientation_convergence_threshold_)
        converged = false;
      error_vector(d) = usc_utilities::clipAbsoluteValue(error_vector(d), max_orientation_error_);
    }

    //ROS_INFO("Jacobian has %d rows", num_jacobian_rows);

    JJt.topLeftCorner(num_jacobian_rows, num_jacobian_rows) =
        world_jacobian.topRows(num_jacobian_rows) * world_jacobian.topRows(num_jacobian_rows).transpose();
    JJt.topLeftCorner(num_jacobian_rows, num_jacobian_rows).diagonal() += damping.head(num_jacobian_rows);

    Jhash.leftCols(num_jacobian_rows) = world_jacobian.topRows(num_jacobian_rows).transpose() *
        JJt.topLeftCorner(num_jacobian_rows, num_jacobian_rows).inverse();

    null_space = MatrixXd::Identity(chain_->num_joints_, chain_->num_joints_) -
        Jhash.leftCols(num_jacobian_rows) * world_jacobian.topRows(num_jacobian_rows);

    // prepare input for cost function:
    cost_function_input->chain_ = chain_;
    cost_function_input->kinematics_info_ = kinematics_info;
    cost_function_input->joint_angles_ = q_out;
    cost_function_input->tool_frame_ = tool_frame;
    cost_function_input->num_dimensions_ = chain_->num_joints_;

    // compute cost function
    cost_function_->getValueAndGradient(cost_function_input, cost_function_value, true, cost_function_gradient, cost_function_state_validity, cost_function_weighted_feature_values);
    null_space_update = - null_space * cost_function_gradient;

    // scale null space update down if needed:
    double max_null_space_update = null_space_update.cwiseAbs().maxCoeff();
    if (max_null_space_update > max_null_space_joint_update_)
    {
      double scale = max_null_space_joint_update_ / max_null_space_update;
      null_space_update *= scale;
      //ROS_INFO("scaled null space update by %f", scale);
    }

    // update best solution thus far
    if (converged && cost_function_state_validity)
    {
      if (my_fk_solver!= fk_solver_)
      {
        my_fk_solver = fk_solver_;
      }
      else
      {
        solution.success = true;
        if (cost_function_value < solution.cost_function_value)
        {
          solution.cost_function_value = cost_function_value;
          solution.joint_angles = q_out;
          solution.tool_frame = tool_frame;
          solution.end_effector_frame = kinematics_info.link_frames_.back();
        }
      }
    }

    //ROS_INFO("converged = %d, cost = %f", converged?1:0, cost_function_value);

    // save prev joint values
    q_prev = q_out;

    // compute update
    delta_theta = Jhash.leftCols(num_jacobian_rows) * error_vector.head(num_jacobian_rows)
        + null_space_update;
    q_out.data += delta_theta;

    // clip at joint limits:
    chain_->clipJointAnglesAtLimits(q_out);
    delta_theta = q_out.data - q_prev.data;

    ++num_iter;
  }
  while(delta_theta.cwiseAbs().maxCoeff() > min_joint_update_threshold_ && num_iter < max_iterations_);

  return solution.success;
}

void ConstrainedIKSolver::getRandomJointAngles(KDL::JntArray& joint_angles) const
{
  int num_joints = chain_->num_joints_;
  joint_angles.resize(num_joints);
  for (int i=0; i<num_joints; ++i)
  {
    double rand01 = (*random_generator_)();
    const double& min = chain_->joints_[i]->limits->lower;
    const double& max = chain_->joints_[i]->limits->upper;
    joint_angles(i) = (max-min)*rand01 + min;
  }
}

}
