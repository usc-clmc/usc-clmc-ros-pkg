/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         arm_movment_handler.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/utils.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pr2_template_based_grasping/arm_movement_handler.h>

using namespace std;
using namespace Eigen;
using namespace geometry_msgs;

namespace pr2_template_based_grasping
{

/* MovementResult */

ArmMovementResult::ArmMovementResult() :
  attempt_valid_(false), object_moved_(false), grasp_successfull_(false)
{
}

/* MovementHandler */

ArmMovementHandler::ArmMovementHandler(ros::NodeHandle& nh, const string& frame_id) :
  move_arm_("/move_right_arm", true), gripper_client_("r_gripper_controller/gripper_action", true)
{
  frame_id_ = frame_id;
  gripper_frame_ = "r_gripper_led_frame";

  zero_pose_.header.frame_id = frame_id_;
  zero_pose_.header.stamp = ros::Time::now();

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    if (!listener.waitForTransform(gripper_frame_, frame_id_, ros::Time(0), ros::Duration(3.0)))
    {
      ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: Waiting for transform timed out.");
    }
    listener.lookupTransform(frame_id_, gripper_frame_, ros::Time(0), transform);
    zero_pose_.pose.position.x = transform.getOrigin().x();
    zero_pose_.pose.position.y = transform.getOrigin().y();
    zero_pose_.pose.position.z = transform.getOrigin().z();
    zero_pose_.pose.orientation.x = transform.getRotation().x();
    zero_pose_.pose.orientation.y = transform.getRotation().y();
    zero_pose_.pose.orientation.z = transform.getRotation().z();
    zero_pose_.pose.orientation.w = transform.getRotation().w();
  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: %s", ex.what());
  }

  while (!move_arm_.waitForServer(ros::Duration(2.0)))
  {
    ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: Waiting for Move Arm Service...");
  }

  ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: Connected to server");
}

ArmMovementHandler::~ArmMovementHandler()
{
}

bool getTransform(tf::StampedTransform& transform, const string& from, const string& to)
{
  tf::TransformListener listener;
  bool result = false;
  try
  {
    if (!listener.waitForTransform(from, to, ros::Time(0), ros::Duration(3.0)))
    {
      ROS_DEBUG_STREAM("pr2_template_based_grasping::ArmMovementHandler: Wait for transform timed out! "
          "Tried to transform from " << from << " to " << to);
    }
    else
    {
      listener.lookupTransform(from, to, ros::Time(0), transform);
      result = true;
    }

  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: %s", ex.what());
  }

  return result;
}

void ArmMovementHandler::transformToWrist(const PoseStamped& input_pose, PoseStamped& output_pose) const
{
  tf::StampedTransform led_to_wrist;
  getTransform(led_to_wrist, gripper_frame_, "r_wrist_roll_link");

  tf::Transform led_to_base;
  tf::poseMsgToTF(input_pose.pose, led_to_base);
  tf::poseTFToMsg(led_to_base * led_to_wrist, output_pose.pose);
}

void ArmMovementHandler::createGripperPoseGoalMsg(const PoseStamped& gripper_pose, arm_navigation_msgs::MoveArmGoal& goal) const
{
  goal.motion_plan_request.group_name = "right_arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = string("");
  goal.planner_service_name = string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  PoseStamped wrist_pose;
  transformToWrist(gripper_pose, wrist_pose);
  desired_pose.header.frame_id = gripper_pose.header.frame_id;
  desired_pose.link_name = "r_wrist_roll_link";
  desired_pose.pose = wrist_pose.pose;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;

  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);
}

bool ArmMovementHandler::executeMovement(arm_navigation_msgs::MoveArmGoal& goal)
{
  bool finished_within_time = false;
  move_arm_.sendGoal(goal);
  finished_within_time = move_arm_.waitForResult(ros::Duration(200.0));
  if (!finished_within_time)
  {
    move_arm_.cancelGoal();
    ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: "
        "Timed out trying to achieve goal");
  }
  else
  {
    actionlib::SimpleClientGoalState state = move_arm_.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
    {
      ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: Action finished: %s",
          state.toString().c_str());

      return true;
    }
    else
    {
      ROS_DEBUG("pr2_template_based_grasping::ArmMovementHandler: Action failed: %s",
          state.toString().c_str());
    }
  }

  return false;
}

void ArmMovementHandler::getApproachPose(const PoseStamped& gripper_pose, PoseStamped& aprch_pose) const
{
  aprch_pose = gripper_pose;
  Vector3d backstep(-0.02, 0, 0);
  Quaterniond orientation;
  orientation.w() = aprch_pose.pose.orientation.w;
  orientation.x() = aprch_pose.pose.orientation.x;
  orientation.y() = aprch_pose.pose.orientation.y;
  orientation.z() = aprch_pose.pose.orientation.z;

  backstep = orientation * backstep;

  aprch_pose.pose.position.x += backstep.x();
  aprch_pose.pose.position.y += backstep.y();
  aprch_pose.pose.position.z += backstep.z();
}

bool ArmMovementHandler::goToZeroPose()
{
  bool succ = false;
  succ = openGripper();

  arm_navigation_msgs::MoveArmGoal goal_pose;
  createGripperPoseGoalMsg(zero_pose_, goal_pose);
  succ = succ && executeMovement(goal_pose);

  return succ;
}

bool ArmMovementHandler::openGripper()
{
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1.0; // Do not limit effort (negative)

  gripper_client_.sendGoal(open);
  gripper_client_.waitForResult();

  return (gripper_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ArmMovementHandler::closeGripper()
{
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = 50.0; // Close gently

  gripper_client_.sendGoal(squeeze);
  gripper_client_.waitForResult();

  return (gripper_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ArmMovementHandler::isObjectInGripper()
{

  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = 100.0;

  gripper_client_.sendGoal(squeeze);
  gripper_client_.waitForResult();

  return (gripper_client_.getResult()->position > 0.002);

}

} //namespace
