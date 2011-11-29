/*
 * jacobian.cpp
 *
 *  Created on: Aug 21, 2011
 *      Author: righetti
 */

#include <jacobian_utilities/jacobian.h>
#include <kdl_parser/kdl_parser.hpp>

namespace jacobian_utilities
{

Jacobian::Jacobian()
{
  initialized_ = false;
}

Jacobian::~Jacobian()
{
}

bool Jacobian::initialize(const std::string& start_link, const std::string& end_link)
{
  ROS_DEBUG("initialize jacobian from link %s to link %s\n", start_link.c_str(), end_link.c_str());
  start_link_name_ = start_link;
  end_link_name_ = end_link;

  ROS_VERIFY(urdf_.initParam("/robot_description"));
  ROS_VERIFY(kdl_parser::treeFromUrdfModel(urdf_, kdl_tree_));
  ROS_VERIFY(kdl_tree_.getChain(start_link, end_link, kdl_chain_));

  num_joints_ = kdl_chain_.getNrOfJoints();
  ROS_DEBUG("using %d joints", num_joints_);

  kdl_jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  kdl_jacobian_.resize(num_joints_);
  kdl_joint_positions_.resize(num_joints_);

  jacobian_ = Eigen::MatrixXd::Zero(6, num_joints_);


  initialized_ = true;
  return initialized_;
}

bool Jacobian::getJacobian(const Eigen::VectorXd& joint_values, Eigen::MatrixXd& jacobian)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(joint_values.size() == num_joints_);

  for(int i=0; i<num_joints_; i++)
  {
    kdl_joint_positions_.data[i] = joint_values[i];
  }

  kdl_jnt_to_jac_solver_->JntToJac(kdl_joint_positions_, kdl_jacobian_);
  jacobian = kdl_jacobian_.data;

  return true;
}

double Jacobian::getManipulabilityMeasure(const Eigen::VectorXd& joint_values)
{
  ROS_ASSERT(initialized_);

  Eigen::MatrixXd jac;

  getJacobian(joint_values, jac);

  Eigen::MatrixXd JJT = jac * jac.transpose();
  return sqrt(JJT.determinant());

}

void Jacobian::getJJTPartialDerivatives(const Eigen::VectorXd& joint_values, double derivative_step,
                                        std::vector< Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6> > >& derivatives)
{
  ROS_ASSERT(initialized_);

  derivatives.resize(num_joints_);

  for(int i=0; i<num_joints_; i++)
  {
    getJJTPartialDerivative(joint_values, derivative_step, i, derivatives[i]);
  }
}

void Jacobian::getJJTPartialDerivative(const Eigen::VectorXd& joint_values, double derivative_step, int derivative_index, Eigen::Matrix<double, 6, 6>& derivative)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(derivative_index>=0 && derivative_index<num_joints_);
  ROS_ASSERT(joint_values.size() == num_joints_);
  ROS_ASSERT(derivative_step > 0);

  Eigen::VectorXd sample_joints = joint_values;
  Eigen::MatrixXd jac;
  Eigen::Matrix<double, 6, 6> JJT_p, JJT_m;

  //get the positive
  sample_joints[derivative_index] += derivative_step;
  getJacobian(sample_joints, jac);
  JJT_p = jac * jac.transpose();

  //get the negative
  sample_joints[derivative_index] -= 2*derivative_step;
  getJacobian(sample_joints, jac);
  JJT_m = jac * jac.transpose();

  derivative = (JJT_p - JJT_m) / (2*derivative_step);
}

void Jacobian::getManipulabilitySqrdPartDerivative(const Eigen::VectorXd& joint_values, double derivative_step, Eigen::VectorXd& part_derivatives)
{
  ROS_ASSERT(initialized_);


  //make a vector of the right size
  part_derivatives = Eigen::VectorXd::Zero(num_joints_);

  //compute the jacobian
  Eigen::MatrixXd jac;
  Eigen::Matrix<double, 6, 6> inv_JJT, JJT;

  getJacobian(joint_values, jac);
  //compute the inverse adding some damping
  JJT = jac * jac.transpose();
  inv_JJT = (JJT + ridge_factor_ * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
  double JJT_det = JJT.determinant();

  Eigen::Matrix<double, 6, 6> JJT_derivative;
  for(int i=0; i<num_joints_; i++)
  {
    getJJTPartialDerivative(joint_values, derivative_step, i, JJT_derivative);
    part_derivatives[i] = JJT_det * (inv_JJT * JJT_derivative).trace();
  }
}

}
