#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>
#include "test_collision_world.h"
#include <tf/transform_datatypes.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <visualization_msgs/Marker.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

class TestOMP
{

public:
  TestOMP();
  ~TestOMP();

  void runTests();

private:

  void addUprightPathConstraint(move_arm_msgs::MoveArmGoal& goal);
  void publishPose(geometry_msgs::Pose& pose, int id);
  void getGoals();
  void goToGoal(int index, bool constrained);
  void goDirectlyToGoal(int index);

  ros::NodeHandle local_nh_;
  ros::NodeHandle node_handle_;
  std::vector<move_arm_msgs::MoveArmGoal> goals_;
  std::vector<geometry_msgs::Pose> poses_;

  std::string planner_service_name_;
  std::string ik_frame_;
  bool use_constraints_;
  bool real_world_;

  stomp_motion_planner::TestCollisionWorld collision_world_;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> joint_traj_action_;
  ros::Publisher goal_pub_;
  ros::Publisher rviz_pub_;
  ros::ServiceClient ik_query_client_;
  ros::ServiceClient ik_client_;
  int num_reps_;

};

TestOMP::TestOMP():
    local_nh_("~"),
    move_arm_("move_right_arm",true),
    joint_traj_action_("r_arm_controller/joint_trajectory_action",true)
{
  goal_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("test_omp/goal_pose", 10, true);
  rviz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("test_omp/visualization_marker", 10);
  ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
  ik_query_client_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  ik_client_ = node_handle_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");

  ros::Duration(1.0).sleep(); // hack to wait for subscribers

  local_nh_.param("planner_service_name", planner_service_name_, std::string("stomp_motion_planner/plan_path"));
  local_nh_.param("use_constraints", use_constraints_, false);
  local_nh_.param("poses_frame", ik_frame_, std::string("/torso_lift_link"));
  local_nh_.param("real_world", real_world_, true);
  local_nh_.param("num_reps", num_reps_, 1);

  move_arm_.waitForServer();
  joint_traj_action_.waitForServer();
  ROS_INFO("Connected to action servers.");

}

void TestOMP::addUprightPathConstraint(move_arm_msgs::MoveArmGoal& goal)
{
  motion_planning_msgs::OrientationConstraint oc;
  oc.header.frame_id = "base_link";
  oc.header.stamp = ros::Time::now();
  oc.link_name = "r_gripper_tool_frame";
  oc.type = oc.HEADER_FRAME;
  oc.absolute_pitch_tolerance = 0.2;
  oc.absolute_roll_tolerance = 0.2;
  oc.absolute_yaw_tolerance = 10.0;
  oc.weight = 1.0;
  btQuaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  tf::quaternionTFToMsg(q, oc.orientation);
  goal.motion_plan_request.path_constraints.orientation_constraints.push_back(oc);
}

void TestOMP::publishPose(geometry_msgs::Pose& pose, int id)
{
  visualization_msgs::Marker msg2;
  msg2.pose = pose;
  msg2.header.frame_id = ik_frame_;
  msg2.header.stamp = ros::Time::now();
  msg2.ns = "test_omp_pose";
  msg2.id = id;
  msg2.type = visualization_msgs::Marker::ARROW;
  msg2.action = visualization_msgs::Marker::ADD;
  msg2.scale.x = 0.15; // length
  msg2.scale.y = 0.10; // dia
  msg2.scale.z = 0.10; // dia
  msg2.color.a = 0.6;
  msg2.color.r = 0.5;
  msg2.color.g = 1.0;
  msg2.color.b = 0.3;
  rviz_pub_.publish(msg2);
  ros::spinOnce();
}

void TestOMP::getGoals()
{

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(ik_query_client_.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(-1);
  }

  // define the service messages
  kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;
//  kinematics_msgs::GetPositionIK::Request  gpik_req;
//  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";
  gpik_req.ik_request.pose_stamped.header.frame_id = ik_frame_;
  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }

  goals_.clear();
  poses_.clear();

  XmlRpc::XmlRpcValue list;
  if (local_nh_.getParam("poses", list))
  {
    for (int32_t i = 0; i < list.size(); ++i)
    {
      XmlRpc::XmlRpcValue pose_xml = list[i];
      geometry_msgs::Pose pose;
      pose.position.x = pose_xml[0];
      pose.position.y = pose_xml[1];
      pose.position.z = pose_xml[2];

      double roll = pose_xml[3];
      double pitch = pose_xml[4];
      double yaw = pose_xml[5];

      btQuaternion q;
      q.setRPY(roll, pitch, yaw);
      tf::quaternionTFToMsg(q, pose.orientation);

      gpik_req.ik_request.pose_stamped.pose = pose;

      // call the IK service:
      if(ik_client_.call(gpik_req, gpik_res))
      {
        if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
        {
//          ROS_INFO("Pose %d: ", i);
//          for(unsigned int j=0; j < gpik_res.solution.joint_state.name.size(); j++)
//          {
//            ROS_INFO("\tJoint: %s %f",gpik_res.solution.joint_state.name[j].c_str(),gpik_res.solution.joint_state.position[j]);
//          }
        }
        else
        {
          ROS_ERROR("Inverse kinematics failed on pose %d", i);
          continue;
        }
      }
      else
      {
        ROS_ERROR("Inverse kinematics service call failed on pose %d", i);
        continue;
      }

      publishPose(pose, i);

      // fill in the goal

      move_arm_msgs::MoveArmGoal goal;

      goal.motion_plan_request.group_name = "right_arm";
      goal.motion_plan_request.num_planning_attempts = 1;
      goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
      goal.motion_plan_request.planner_id= std::string("");
      goal.planner_service_name = planner_service_name_;

      motion_planning_msgs::Constraints constraints;
      for(unsigned int j=0; j < gpik_res.solution.joint_state.name.size(); j++)
      {
        motion_planning_msgs::JointConstraint jc;
        jc.joint_name = gpik_res.solution.joint_state.name[j];
        jc.position = gpik_res.solution.joint_state.position[j];
        jc.tolerance_above = 0.1;
        jc.tolerance_below = 0.1;
        goal.motion_plan_request.goal_constraints.joint_constraints.push_back(jc);
      }
      //if (use_constraints_)
      //  addUprightPathConstraint(goal);
      goals_.push_back(goal);
      poses_.push_back(pose);

    }
  }

}

void TestOMP::goToGoal(int index, bool constrained)
{

  // first publish the goal to rviz
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = poses_[index];
  pose_stamped.header.frame_id = ik_frame_;
  pose_stamped.header.stamp = ros::Time::now();
  goal_pub_.publish(pose_stamped);
  ros::spinOnce();

  move_arm_msgs::MoveArmGoal goal = goals_[index];
  if (constrained)
    addUprightPathConstraint(goal);

  bool succeeded = false;

  while (!succeeded && ros::ok())
  {
    bool finished_within_time = false;
    move_arm_.sendGoal(goal);
    finished_within_time = move_arm_.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm_.cancelGoal();
      ROS_INFO("Timed out achieving goal");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm_.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
      {
        ROS_INFO("Action finished: %s",state.toString().c_str());
        succeeded = true;
      }
      else
      {
        ROS_INFO("Action failed: %s",state.toString().c_str());
        goDirectlyToGoal(index);
        succeeded = true;
      }
    }
  }
}

void TestOMP::goDirectlyToGoal(int index)
{
  move_arm_msgs::MoveArmGoal goal = goals_[index];
  // create the goal
  pr2_controllers_msgs::JointTrajectoryGoal jt_goal;

  jt_goal.trajectory.points.resize(1);
  for (int i=0; i<int(goal.motion_plan_request.goal_constraints.joint_constraints.size()); ++i)
  {
    jt_goal.trajectory.joint_names.push_back(goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name);
    jt_goal.trajectory.points[0].positions.push_back(goal.motion_plan_request.goal_constraints.joint_constraints[i].position);
    jt_goal.trajectory.points[0].velocities.push_back(0.0);
    jt_goal.trajectory.points[0].accelerations.push_back(0.0);
  }
  jt_goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

  bool finished_within_time = false;
  joint_traj_action_.sendGoal(jt_goal);
  finished_within_time = joint_traj_action_.waitForResult(ros::Duration(200.0));
  if (!finished_within_time)
  {
    joint_traj_action_.cancelGoal();
    ROS_INFO("Timed out achieving goal");
  }
  else
  {
    actionlib::SimpleClientGoalState state = joint_traj_action_.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if(success)
    {
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action failed: %s",state.toString().c_str());
  }
}

void TestOMP::runTests()
{

  // instantiate the collision world
  collision_world_.removeAll();

  ros::spinOnce();
  ros::Duration(1.0).sleep();

  collision_world_.createObjectsFromParamServer(local_nh_);
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  getGoals();

  int num_goals = goals_.size();
  int cur_position=-1;

  for (int rep=0; rep<num_reps_; ++rep)
  {
    for (int from=0; from < num_goals-1; ++from)
    {
      for (int to=from+1; to < num_goals; ++to)
      {
        if (!node_handle_.ok())
          return;
        if (cur_position != from)
        {
          if (real_world_)
            goToGoal(from, false);
          else
            goDirectlyToGoal(from);
        }
        goToGoal(to, use_constraints_);
        goToGoal(from, use_constraints_);
        cur_position = from;
      }
    }
  }
}

TestOMP::~TestOMP()
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_omp");
  TestOMP test_omp;
  test_omp.runTests();
  ros::shutdown();
  return 0;
}
