/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         arm_movment_handler.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef ARM_MOVEMENT_HANDLER_H_
#define ARM_MOVEMENT_HANDLER_H_

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <actionlib/client/simple_action_client.h>

namespace pr2_template_based_grasping
{

struct ArmMovementResult
{
public:
  ArmMovementResult();

  bool attempt_valid_;
  bool object_moved_;
  bool grasp_successfull_;
};

class ArmMovementHandler
{
public:

  ArmMovementHandler(ros::NodeHandle& nh, const std::string& frame_id);
  ~ArmMovementHandler();

  bool goToZeroPose();
  bool isObjectInGripper();
  void transformToWrist(const geometry_msgs::PoseStamped& input_pose,
      geometry_msgs::PoseStamped& output_pose) const;
  void getApproachPose(const geometry_msgs::PoseStamped& gripper_pose,
      geometry_msgs::PoseStamped& aprch_pose) const;

private:

  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> gripper_client_;
  ros::Publisher coll_obj_pub_;
  geometry_msgs::PoseStamped zero_pose_;
  std::string frame_id_, gripper_frame_;

  void createGripperPoseGoalMsg(const geometry_msgs::PoseStamped& gripper_pose,
      arm_navigation_msgs::MoveArmGoal& goal) const;
  bool executeMovement(arm_navigation_msgs::MoveArmGoal& goal);
  bool openGripper();
  bool closeGripper();
};

} //namespace
#endif /* ARM_MOVEMENT_HANDLER_H_ */
