#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <robot_info/robot_info.h>
#include <usc_utilities/assert.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/dmp_joint_position_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_controller.h>
#include <pr2_dynamic_movement_primitive_controller/dmp_controller_implementation.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

PLUGINLIB_DECLARE_CLASS(pr2_dynamic_movement_primitive_controller, DMPJointPositionController, pr2_dynamic_movement_primitive_controller::DMPJointPositionController, pr2_controller_interface::Controller)

namespace pr2_dynamic_movement_primitive_controller
{

bool DMPJointPositionController::init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& node_handle)
{
//	/* DEBUG */ROS_INFO("Initializing...");

	std::vector<std::string> controlled_joint_names;
	controlled_joint_names.clear();

//	/* DEBUG */ROS_INFO("DEBUG: Initializing_55");

    /*  initialize joint position controllers */
    joint_position_controllers_.clear();
    std::string joint_names_string;
    node_handle.param<std::string> ("joint_names", joint_names_string, "");

    std::stringstream ss(joint_names_string);
    std::string joint_name;

//	/* DEBUG */ROS_INFO("DEBUG: Initializing_4");

    while (ss >> joint_name)
    {
    	controlled_joint_names.push_back(joint_name);

        ros::NodeHandle joint_node_handle(node_handle, joint_name);
        JointPositionController joint_controller;
        if (!joint_controller.init(robot_state, joint_node_handle))
        {
            ROS_ERROR("Could not initialize joint controller for joint %s.", joint_name.c_str());
            return false;
        };
        joint_position_controllers_.push_back(joint_controller);
    };

//	/* DEBUG */ROS_INFO("DEBUG: Initializing_3");

    std::string class_name;
    node_handle.param<std::string> ("dmp_implementation", class_name, "");
    if (class_name == "ICRA2009DMPControllerImplementation")
        dmp_controller_.reset(new DMPControllerImplementation<dmp::ICRA2009DMP>());
    else
    	ROS_ERROR("Could not figure out witch DMPController implementation to chose.");

//	/* DEBUG */ROS_INFO("DEBUG: Initializing_2");

    /* DEBUG: ROS_VERIFY ??? */
    /* DEBUG: DEFINE PROPER CONTROLLER NAME */
    assert(dmp_controller_->initialize(node_handle.getNamespace(), controlled_joint_names));

    // initialize member
    num_joints_ = controlled_joint_names.size();
    desired_positions_ = Eigen::VectorXd::Zero(num_joints_);
    desired_velocities_ = Eigen::VectorXd::Zero(num_joints_);
    desired_accelerations_ = Eigen::VectorXd::Zero(num_joints_);

//	/* DEBUG */ROS_INFO("DEBUG: Initializing_1");

    return true;
}

void DMPJointPositionController::starting()
{

  /* DEBUG */ROS_INFO("Starting...");

  for (int i = 0; i < static_cast<int>(joint_position_controllers_.size()); ++i)
  {
      joint_position_controllers_[i].starting();
  }

  /* DEBUG: ATTENTION WITH INIT OF JOINT POS CTRL */
  holdPositions();

}

void DMPJointPositionController::update()
{

  /* DEBUG *///ROS_INFO("Updating...");



  if (dmp_controller_->newDMPReady())
  {
    // set start of DMP to current desired position
	/* DEBUG: ROS_VERIFY ??? */
    ROS_VERIFY(dmp_controller_->changeDMPStart(desired_positions_));

  }

  if (dmp_controller_->isRunning(desired_positions_, desired_velocities_, desired_accelerations_))
  {
    setDesiredState();
  }
  else
  {
    holdPositions();
  }


  for (int i = 0; i < static_cast<int>(joint_position_controllers_.size()); i++)
  {
      joint_position_controllers_[i].update();
  }
}

// REAL-TIME REQUIREMENTS
void DMPJointPositionController::setDesiredState()
{
    for (int i = 0; i < num_joints_; i++)
    {
        joint_position_controllers_[i].setCommand(desired_positions_(i));
    }
}

void DMPJointPositionController::holdPositions()
{
	getDesiredPosition();
	desired_velocities_.setZero(num_joints_);
	desired_accelerations_.setZero(num_joints_);
	setDesiredState();
}

// REAL-TIME REQUIREMENTS
void DMPJointPositionController::getDesiredPosition()
{
    for (int local_joint = 0; local_joint < num_joints_; ++local_joint)
    {
      desired_positions_(local_joint) = joint_position_controllers_[local_joint].getJointPosition();
    }
}

}
