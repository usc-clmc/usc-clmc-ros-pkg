/*
 * test_jacobian.cpp
 *
 *  Created on: Aug 22, 2011
 *      Author: righetti
 */


#include <ros/ros.h>
#include <boost/thread.hpp>

#include <Eigen/Eigen>

#include <jacobian_utilities/jacobian.h>
#include <arm_controller_interface/robot_monitor.h>


void spinner()
{
  ROS_INFO("spinning.");
  ros::MultiThreadedSpinner mts;
  //  mts.spin();
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  arm_controller_interface::init();

  //start spinning
  boost::thread spin_thread(spinner);

  //get a monitor
  arm_controller_interface::RobotMonitor monitor;
  arm_controller_interface::RobotDesiredMonitor des_monitor;

  ros::Duration(2.0).sleep();

  //get a jacobian
  jacobian_utilities::Jacobian jac_utils;
  jac_utils.initialize();

  std::vector<double> joint_states, joint_des_states;
  monitor.getRightArmJointPositions(joint_states);
  des_monitor.getRightArmJointPositions(joint_des_states);

  Eigen::Map<Eigen::VectorXd> eigen_joint_states(&joint_states[0], joint_states.size());
  Eigen::Map<Eigen::VectorXd> eigen_joint_des_states(&joint_des_states[0], joint_states.size());
  Eigen::MatrixXd jacobian, des_jacobian;

  jac_utils.getJacobian(eigen_joint_states, jacobian);
  jac_utils.getJacobian(eigen_joint_des_states, des_jacobian);

  std::cout << "the jacobian is" << std::endl << jacobian << std::endl;
  std::cout << "and the manipulability measure is: " << jac_utils.getManipulabilityMeasure(eigen_joint_states) << std::endl;

  std::cout << "the jacobian is" << std::endl << des_jacobian << std::endl;
  std::cout << "and the manipulability measure is: " << jac_utils.getManipulabilityMeasure(eigen_joint_des_states) << std::endl;


  //compute the manipulability partial derivative
  ros::Time t1, t2;
  t1 = ros::Time::now();
  double derivative_step = 10e-3;
  Eigen::VectorXd part_derivative;
  jac_utils.getManipulabilitySqrdPartDerivative(eigen_joint_states, derivative_step, part_derivative);
  t2 = ros::Time::now();
  std::cout << "the manipulability part derivative is" << std::endl << part_derivative << std::endl;
  std::cout << "took " << double((t2 - t1).toNSec()) /1000.0/1000.0 << " [ms]" << std::endl;

  //compute the partial derivatives of JJT
  t1 = ros::Time::now();
  std::vector< Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6> > > derivatives;
  jac_utils.getJJTPartialDerivatives(eigen_joint_states, derivative_step, derivatives);
  t2 = ros::Time::now();

  std::cout << "the partial derivatives for JJT are:" << std::endl;
  for(int i=0; i<(int)derivatives.size(); i++)
  {
    std::cout<< " joint " << i << std::endl << derivatives[i] << std::endl;
  }
  std::cout << "took " << double((t2 - t1).toNSec()) /1000.0/1000.0 << " [ms]" << std::endl;

  return 0;

}
