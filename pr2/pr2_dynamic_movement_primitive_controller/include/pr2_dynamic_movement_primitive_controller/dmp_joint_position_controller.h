/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   dmp_controller.h

  \author Alexander Herzog, Peter Pastor
  \date   Mar 1, 2011

 *********************************************************************/

#ifndef DMP_JOINT_POSITION_CONTROLLER_H_
#define DMP_JOINT_POSITION_CONTROLLER_H_

// system include
#include <boost/shared_ptr.hpp>
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>

// ros include
#include <ros/ros.h>
#include <control_toolbox/pid.h>

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

// local include
#include <pr2_dynamic_movement_primitive_controller/joint_position_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_controller.h>

#include <Eigen/Eigen>

// local include

namespace pr2_dynamic_movement_primitive_controller
{

class DMPJointPositionController : public pr2_controller_interface::Controller
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! constructor
     *
     */
    DMPJointPositionController() {};

    /*! descructor
     *
     */
    virtual ~DMPJointPositionController() {};

    /*!
     * @param robot_state
     * @param node
     * @return
     */
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node);

    /*!
     * @return
     */
    void starting();

    /*!
     *
     */
    void update();

    /*!
     * REAL-TIME REQUIREMENTS
     */
    void setDesiredState();

    /*!
     * REAL-TIME REQUIREMENTS
     */
    void holdPositions();

    /*!
     */
    void getDesiredPosition();


private:

    pr2_mechanism_model::JointState* joint_state_;
    int num_joints_;

    Eigen::VectorXd desired_positions_;
    Eigen::VectorXd desired_velocities_;
    Eigen::VectorXd desired_accelerations_;


    std::vector<JointPositionController> joint_position_controllers_;

    // DMPController
    boost::shared_ptr<DMPController> dmp_controller_;
};

}

#endif /* DMP_JOINT_POSITION_CONTROLLER_H_ */
