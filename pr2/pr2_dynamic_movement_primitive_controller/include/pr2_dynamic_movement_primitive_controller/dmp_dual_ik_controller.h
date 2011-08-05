/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks    ...

 \file   dmp_dual_ik_controller.h

 \author Peter Pastor
 \date   Jan 12, 2011

 *********************************************************************/

#ifndef DMP_DUAL_IK_CONTROLLER_H_
#define DMP_DUAL_IK_CONTROLLER_H_

// system includes
#include <boost/shared_ptr.hpp>

// ros includes
#include <rosrt/rosrt.h>
#include <pr2_controller_interface/controller.h>

#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>

// local includes
#include <pr2_dynamic_movement_primitive_controller/dmp_controller.h>
#include <pr2_dynamic_movement_primitive_controller/cartesian_twist_controller_ik_with_nullspace_optimization.h>

/*!
 */
namespace pr2_dynamic_movement_primitive_controller
{

/*! \class controller that uses cartesian space set points and sends them to the pose twist controller.
 */
class DMPDualIkController : public pr2_controller_interface::Controller
{

  typedef CartesianTwistControllerIkWithNullspaceOptimization ChildController;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! Constructor
   */
  DMPDualIkController();

  /*! Destructor
   */
  virtual ~DMPDualIkController() {};

  /*!
   * @param robot_state
   * @param node_handle
   * @return
   */
  bool init(pr2_mechanism_model::RobotState* robot_state,
            ros::NodeHandle& node_handle);

  /*!
   */
  void starting();

  /*!
   */
  void update();

  /*!
   */
  void stopping();

  /*!
   * @return
   */
  bool initXml(pr2_mechanism_model::RobotState* robot,
               TiXmlElement* config);

  /*!
   * @return
   */
  std::string getVariableNamesKeyWord() const;
  std::vector<int> getNumDofs() const;

private:

  /*!
   * @param trajectory_point
   * @param movement_finished
   * @param execution_duration
   * @param num_samples
   * @return
   */
  bool transformCommand(Eigen::VectorXd& trajectory_point,
                        bool& movement_finished,
                        const double execution_duration,
                        const int num_samples);

  /*!
   * @return
   */
  bool readParameters();

  /*!
   * @return
   */
  bool initRTPublisher();

  /*!
   * @param handle_namespace
   * @param controller_handle_namespace
   * @return
   */
  bool getArmRelatedVariables(const std::string& handle_namespace,
                              std::string& controller_handle_namespace);

  /*!
   */
  bool initialized_;

  /*! robot structure
   */
  pr2_mechanism_model::RobotState* robot_state_;

  /*!
   */
  std::string root_name_;

  /*!
   */
  ros::NodeHandle node_handle_;

  /*! IK controller
   */
  boost::shared_ptr<ChildController> cart_controller_;

  /*!
   */
  void visualize();

  /*!
   * @param input_vector
   * @param output_vector
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool adjustVariables(const Eigen::VectorXd& input_vector, Eigen::VectorXd& output_vector);

  /*!
   * REAL-TIME REQUIREMENTS
   * @return True on success, otherwise False
   */
  bool setDesiredState();

  /*!
   * REAL-TIME REQUIREMENTS
   * @return True on success, otherwise False
   */
  bool holdPositions();

  /*!
   * REAL-TIME REQUIREMENTS
   * @return True on success, otherwise False
   */
  bool getDesiredPosition();

  /*!
   */
  int publishing_rate_;
  int publishing_counter_;
  int publisher_buffer_size_;

//  /*!
//   */
//  Eigen::Vector3d actual_endeffector_linear_twist_;
//  Eigen::Vector3d desired_endeffector_linear_twist_;

  /*!
   */
  boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_actual_arrow_publisher_;
  boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_desired_arrow_publisher_;
  boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > pose_actual_publisher_;
  boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > pose_desired_publisher_;

  int visualization_line_counter_;
  int visualization_line_rate_;
  int visualization_line_max_points_;
  int visualization_line_points_index_;
  boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_actual_line_publisher_;
  boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_desired_line_publisher_;

  /*!
   */
  bool keep_restposture_fixed_for_testing_;

  /*!
   */
  bool last_frame_set_;
  KDL::Frame last_frame_;
  KDL::Frame current_frame_;
  KDL::Twist current_twist_;

  /*!
   */
  int num_joints_;
  Eigen::VectorXd desired_positions_;
  Eigen::VectorXd desired_velocities_;
  Eigen::VectorXd desired_accelerations_;

  Eigen::VectorXd goal_;
  Eigen::VectorXd start_;
  Eigen::VectorXd local_vector_;

  // DMPController
  boost::shared_ptr<DMPController> dmp_controller_;

  bool execution_error_;

  /*!
   */
  // pr2_tasks_transforms::TaskTransforms task_frame_transformer_;

  bool isIdle()
  {
    return dmp_controller_->isIdle();
  }

};

}

#endif /* DMP_DUAL_IK_CONTROLLER_H_ */
