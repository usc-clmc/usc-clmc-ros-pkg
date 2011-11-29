/*
 * jacobian.h
 *
 *  Created on: Aug 21, 2011
 *      Author: righetti
 */

#ifndef JACOBIAN_H_
#define JACOBIAN_H_

#include <ros/ros.h>
#include <usc_utilities/assert.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <robot_info/robot_info.h>

#include <vector>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace jacobian_utilities
{
class Jacobian
{
public:
  Jacobian();
  virtual ~Jacobian();

  bool initialize(const std::string& start_link = "BASE", const std::string& end_link = "R_PALM");

  bool getJacobian(const Eigen::VectorXd& joint_values, Eigen::MatrixXd& jacobian);

  double getManipulabilityMeasure(const Eigen::VectorXd& joint_values);

  void getJJTPartialDerivatives(const Eigen::VectorXd& joint_values, double derivative_step,
                                std::vector< Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6> > >& derivatives);

  void getJJTPartialDerivative(const Eigen::VectorXd& joint_values, double derivative_step, int derivative_index, Eigen::Matrix<double, 6, 6>& derivative);

  void getManipulabilitySqrdPartDerivative(const Eigen::VectorXd& joint_values, double derivative_step, Eigen::VectorXd& part_derivatives);

private:

  static const double ridge_factor_ = 10e-6;

  ros::NodeHandle node_handle_;

  bool initialized_;
  std::string start_link_name_;
  std::string end_link_name_;


  urdf::Model urdf_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray kdl_joint_positions_;
  KDL::Jacobian kdl_jacobian_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_jnt_to_jac_solver_;

  int num_joints_;

  Eigen::MatrixXd jacobian_;
};

}

#endif /* JACOBIAN_H_ */
