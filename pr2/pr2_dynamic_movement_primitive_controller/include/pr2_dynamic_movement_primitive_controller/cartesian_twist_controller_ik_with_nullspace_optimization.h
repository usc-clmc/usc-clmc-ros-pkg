/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   cartesian_twist_controller_ik_with_nullspace_optimization.h

  \author Peter Pastor
  \date   Jan 12, 2011

 *********************************************************************/


#ifndef CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_
#define CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_

// system includes
#include <vector>
#include <boost/scoped_ptr.hpp>

// ros includes
#include <ros/ros.h>

#include <rosrt/rosrt.h>

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Core>

#include <geometry_msgs/Twist.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <tf/transform_datatypes.h>

#include <filters/transfer_function.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/joint_position_controller.h>

#include <pr2_dynamic_movement_primitive_controller/JointPositionVelocityStamped.h>
#include <pr2_dynamic_movement_primitive_controller/PoseTwistStamped.h>
#include <pr2_dynamic_movement_primitive_controller/NullspaceTermStamped.h>
// #include <pr2_dynamic_movement_primitive_controller/ControllerStatus.h>

namespace pr2_dynamic_movement_primitive_controller
{

class CartesianTwistControllerIkWithNullspaceOptimization : public pr2_controller_interface::Controller
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   */
  CartesianTwistControllerIkWithNullspaceOptimization();
  virtual ~CartesianTwistControllerIkWithNullspaceOptimization() {};

  /*!
   * @param robot_state
   * @param node_handle
   * @return
   */
  bool init(pr2_mechanism_model::RobotState *robot_state,
            ros::NodeHandle &node_handle);

  /*!
   * @return
   */
  void starting();

  /*!
   */
  void update();

  /*! input of the cartesian twist controller
   */
  KDL::Frame kdl_pose_desired_;
  KDL::Twist kdl_twist_desired_;

  /*! output
   */
  KDL::Frame kdl_pose_measured_;
  KDL::Twist kdl_twist_measured_;

  KDL::Frame kdl_real_pose_measured_;

  /*! only for debugging...
   */
  KDL::JntArray kdl_current_joint_positions_;
  KDL::JntArrayVel kdl_current_joint_velocities_;

  KDL::JntArray kdl_desired_joint_positions_;

  /*! input to the controller for the nullspace optimization part
   */
  Eigen::VectorXd rest_posture_joint_configuration_;

private:

  /*!
   * @return
   */
  bool initMechanismChain();

  /*!
   * @return
   */
  bool readParameters();

  /*!
   * @return
   */
  bool initCartesianPidControllers();
  /*!
   * @return
   */
  bool initNullspacePidControllers();

  /*!
   * @return
   */
  bool initRTPublisher();

  /*! robot description
   */
  pr2_mechanism_model::RobotState *robot_state_;
  pr2_mechanism_model::Chain mechanism_chain_;

  /*!
   */
  ros::NodeHandle node_handle_;

  /*!
   */
  KDL::Twist kdl_twist_error_;
  KDL::Chain kdl_chain_;
  KDL::Jacobian kdl_chain_jacobian_;

  /*!
   */
  boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  /*!
   */
  double damping_;

  /*!
   */
  ros::Time last_time_;
  ros::Duration dt_;

  /*!
   */
  double ff_trans_;
  double ff_rot_;

  /*! feedback pid controllers (translation and rotation)
   */
  std::vector<control_toolbox::Pid> cartesian_fb_pid_controllers_;

  /*! feedback pid controllers (nullspace)
   */
  std::vector<control_toolbox::Pid> nullspace_fb_pid_controllers_;

  /*!
   */
  std::vector<JointPositionController> joint_position_controllers_;

  /*!
   */
  int num_joints_;

  Eigen::VectorXd eigen_desired_cartesian_velocities_;

  Eigen::VectorXd eigen_desired_joint_positions_;
  Eigen::VectorXd eigen_desired_joint_velocities_;

  Eigen::MatrixXd eigen_chain_jacobian_;

  Eigen::MatrixXd eigen_jac_times_jac_transpose_;
  Eigen::MatrixXd eigen_jjt_inverse_;
  Eigen::MatrixXd eigen_jac_pseudo_inverse_;
  Eigen::MatrixXd eigen_identity_;

  Eigen::VectorXd eigen_nullspace_term_;
  Eigen::MatrixXd eigen_nullspace_projector_;
  Eigen::VectorXd eigen_nullspace_error_;

  /*!
   */
  filters::MultiChannelTransferFunctionFilter<double> pose_filter_;
  std::vector<double> pose_unfiltered_data_;
  std::vector<double> pose_filtered_data_;

  /*!
   */
  int publisher_rate_;
  int publisher_counter_;
  int publisher_buffer_size_;
  int header_sequence_number_;

  /*!
   * @param node_handle
   * @param robot
   * @param joint_position_controllers
   * @return
   */
  static bool initJointPositionController(ros::NodeHandle node_handle,
                                          pr2_mechanism_model::RobotState *robot,
                                          std::vector<JointPositionController>& joint_position_controllers);

  /*!
   */
  boost::shared_ptr<rosrt::Publisher<pr2_dynamic_movement_primitive_controller::PoseTwistStamped> > pose_twist_desired_publisher_;
  boost::shared_ptr<rosrt::Publisher<pr2_dynamic_movement_primitive_controller::PoseTwistStamped> > pose_twist_actual_publisher_;
  boost::shared_ptr<rosrt::Publisher<pr2_dynamic_movement_primitive_controller::NullspaceTermStamped> > nullspace_term_publisher_;
  // boost::shared_ptr<rosrt::Publisher<pr2_dynamic_movement_primitive_controller::ControllerStatus> > controller_status_publisher_;

  /*!
   */
  void publish();

};

}

#endif /* CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_ */
