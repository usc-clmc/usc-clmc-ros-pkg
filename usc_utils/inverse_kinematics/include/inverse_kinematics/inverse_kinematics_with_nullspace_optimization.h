/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		inverse_kinematics_with_nullspace_optimization.h

  \author	Peter Pastor
  \date		Jan 24, 2011

 *********************************************************************/

#ifndef INVERSE_KINEMATICS_WITH_NULLSPACE_OPTIMIZATION_H_
#define INVERSE_KINEMATICS_WITH_NULLSPACE_OPTIMIZATION_H_

// system includes
#include <vector>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Eigen>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <usc_utilities/assert.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_utilities/robot_pose_visualizer.h>
#include <usc_utilities/kdl_chain_wrapper.h>
#include <usc_utilities/timer.h>

// local includes

namespace inverse_kinematics
{

class InverseKinematicsWithNullspaceOptimization
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! Constructor
   */
  InverseKinematicsWithNullspaceOptimization(const std::string& robot_part_name) :
      initialized_(false), robot_part_name_(robot_part_name), node_handle_(ros::NodeHandle()), robot_visualizer_(
          "/inverse_kinematics/ik_wn_" + robot_part_name) {};

  /*! Destructor
   */
  virtual ~InverseKinematicsWithNullspaceOptimization() {};

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool initialize(ros::NodeHandle node_handle = ros::NodeHandle("/LearningFromDemonstration"));

  /*!
   * @param start_link
   * @param end_link
   * @return True on success, otherwise False
   */
  bool initialize(const std::string& start_link,
                  const std::string& end_link);

  /*!
   * @param poses
   * @param rest_postures
   * @param joint_angle_seed
   * @param joint_angles
   * @return True on success, otherwise False
   */
  bool ik(const std::vector<geometry_msgs::PoseStamped>& poses,
          const std::vector<Eigen::VectorXd>& rest_postures,
          const Eigen::VectorXd& joint_angle_seed,
          std::vector<Eigen::VectorXd>& joint_angles);

  /*!
   * @return
   */
  std::string getStartLinkName() const
  {
    ROS_VERIFY(initialized_);
    return start_link_name_;
  }
  /*!
   * @return
   */
  std::string getEndLinkName() const
  {
    ROS_VERIFY(initialized_);
    return end_link_name_;
  }

private:

  bool initialized_;
  std::string robot_part_name_;
  std::string start_link_name_;
  std::string end_link_name_;
  std::string base_link_name_;

  std::string first_link_name_;

  int num_joints_;
  KDL::Chain chain_;

  usc_utilities::Timer timer_;

  // for debugging only:
  ros::NodeHandle node_handle_;
  visualization_utilities::RobotPoseVisualizer robot_visualizer_;
  ros::Publisher target_pose_visualizer_;

  // used in computeIkWithNullspaceOptimization
  boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  KDL::JntArray kdl_current_joint_positions_;
  KDL::JntArray kdl_previous_joint_positions_;

  KDL::Jacobian kdl_chain_jacobian_;

  KDL::Frame kdl_desired_pose_;
  KDL::Frame kdl_previous_pose_;

  KDL::Twist kdl_twist_error_;
  KDL::Twist kdl_twist_desired_;

  Eigen::VectorXd eigen_nullspace_error_;
  Eigen::VectorXd eigen_nullspace_term_;
  Eigen::MatrixXd eigen_nullspace_projector_;

  Eigen::MatrixXd eigen_chain_jacobian_;
  Eigen::MatrixXd eigen_jac_times_jac_transpose_;
  Eigen::MatrixXd eigen_jjt_inverse_;
  Eigen::MatrixXd eigen_jac_pseudo_inverse_;
  Eigen::MatrixXd eigen_identity_;

  Eigen::VectorXd eigen_desired_cartesian_velocities_;
  Eigen::VectorXd eigen_desired_joint_positions_;
  Eigen::VectorXd eigen_desired_joint_velocities_;

  // debugging...
  boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
  boost::scoped_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  KDL::JntArray seed_;
  KDL::JntArray solution_;

};

}

#endif /* INVERSE_KINEMATICS_WITH_NULLSPACE_OPTIMIZATION_H_ */
