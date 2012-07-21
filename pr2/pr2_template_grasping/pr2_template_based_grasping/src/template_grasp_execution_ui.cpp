/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         template_grasp_execution_ui.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_template_based_grasping/arm_movement_handler.h>
#include <pr2_template_based_grasping/head_movement_handler.h>

//// TEMPLATE GRASPING CODE BEGIN: code for planning service and related messages ////
#include <pr2_template_based_grasping/PlanningFeedback.h>
#include <object_manipulation_msgs/GraspPlanning.h>
//// TEMPLATE GRASPING CODE END: code for planning service and related messages ////

using namespace std;
using namespace pr2_template_based_grasping;

int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "template_grasp_execution_ui");
  ros::NodeHandle nh;

  //set service and action names
  const string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
  const string COLLISION_PROCESSING_SERVICE_NAME =
      "/tabletop_collision_map_processing/tabletop_collision_map_processing";
  const string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
  const string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
  const string HEAD_ACTION_NAME = "/head_traj_controller/point_head_action";

  //create service and action clients
  ros::ServiceClient object_detection_srv;
  ros::ServiceClient collision_processing_srv;
  actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> pickup_client(PICKUP_ACTION_NAME, true);
  actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> place_client(PLACE_ACTION_NAME, true);
//  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> head_client(HEAD_ACTION_NAME, true);
  pr2_controllers_msgs::PointHeadGoal head_goal;
  ArmMovementHandler mov_handl(nh, "/base_link");

//// TEMPLATE GRASPING CODE BEGIN: connect to planning service ////
  const string GRASP_PLANNING_SERVICE_NAME = "/pr2_template_grasp_planner";
  const string GRASP_FEEDBACK_SERVICE_NAME = "/pr2_template_grasp_planner_feedback";
  ros::ServiceClient grasp_planning_client;
  ros::ServiceClient grasp_planning_feedback_client;

  //wait for grasp planning client
  while (!ros::service::waitForService(GRASP_PLANNING_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for grasp planning service to come up");
  }
  if (!nh.ok())
    return 0;
  grasp_planning_client = nh.serviceClient<object_manipulation_msgs::GraspPlanning> (
      GRASP_PLANNING_SERVICE_NAME, true);

  //wait for grasp planning feedback client
  while (!ros::service::waitForService(GRASP_FEEDBACK_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for grasp planning feedback service to come up");
  }
  if (!nh.ok())
    return 0;
  grasp_planning_feedback_client = nh.serviceClient<PlanningFeedback> (
      GRASP_FEEDBACK_SERVICE_NAME);
//// TEMPLATE GRASPING CODE END: connect to planning service ////

  //wait for detection client
  while (!ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for object detection service to come up");
  }
  if (!nh.ok())
    return 0;
  object_detection_srv = nh.serviceClient<tabletop_object_detector::TabletopDetection> (
      OBJECT_DETECTION_SERVICE_NAME, true);

  //wait for collision map processing client
  while (!ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for collision processing service to come up");
  }
  if (!nh.ok())
    return 0;
  collision_processing_srv = nh.serviceClient<tabletop_collision_map_processing::
      TabletopCollisionMapProcessing> (COLLISION_PROCESSING_SERVICE_NAME, true);

  //wait for pickup client
  pickup_client.cancelAllGoals();
  while (!pickup_client.waitForServer(ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
  }
  if (!nh.ok())
    return 0;

  //wait for place client
  while (!place_client.waitForServer(ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
  }
  if (!nh.ok())
    return 0;

  while (ros::ok())
  {
    ROS_INFO_STREAM("Enter 'go' to start the pickup process or" << endl <<
        "'zero' to move the arm to it's initial position or" << endl <<
        "'q' to quit.");

    string uinp;
    cin >> uinp;
    if (uinp == "q")
    {
      break;
    }
    else if (uinp == "go")
    {
      //call the tabletop detection
      ROS_INFO("Calling tabletop detector");
      tabletop_object_detector::TabletopDetection detection_call;
      detection_call.request.return_clusters = true;
      detection_call.request.return_models = true;
      if (!object_detection_srv.call(detection_call))
      {
        ROS_ERROR("Tabletop detection service failed");
        continue;
      }
      if (detection_call.response.detection.result != detection_call.response.detection.SUCCESS)
      {
        ROS_ERROR("Tabletop detection returned error code %d",
            detection_call.response.detection.result);
        continue;
      }
      if (detection_call.response.detection.clusters.empty()
          && detection_call.response.detection.models.empty())
      {
        ROS_ERROR("The tabletop detector detected the table, but found no objects");
        continue;
      }

      //call collision map processing
      ROS_INFO("Calling collision map processing");
      tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_call;
      processing_call.request.detection_result = detection_call.response.detection;
      processing_call.request.reset_collision_models = true;
      processing_call.request.reset_attached_models = true;
      processing_call.request.desired_frame = "base_link";
      if (!collision_processing_srv.call(processing_call))
      {
        ROS_ERROR("Collision map processing service failed");
        continue;
      }
      if (processing_call.response.graspable_objects.empty())
      {
        ROS_ERROR("Collision map processing returned no graspable objects");
        continue;
      }

//// TEMPLATE GRASPING CODE BEGIN: request grasps for detected point-cloud cluster ////
      ROS_INFO("Calling template grasp planner");
      object_manipulation_msgs::GraspPlanning grasp_planning_call;
      grasp_planning_call.request.target.cluster = detection_call.response.detection.clusters.at(0);
      if (!grasp_planning_client.call(grasp_planning_call))
      {
        ROS_ERROR("Grasp planning service failed.");
        continue;
      }
//// TEMPLATE GRASPING CODE END: request grasps for detected point-cloud cluster ////

      //call object pickup
      ROS_INFO("Calling the pickup action");
      object_manipulation_msgs::PickupGoal pickup_goal;

//// TEMPLATE GRASPING CODE BEGIN: give computed grasps to manipulation pipeline ////
      pickup_goal.desired_grasps = grasp_planning_call.response.grasps;
//// TEMPLATE GRASPING CODE END: give computed grasps to manipulation pipeline ////

      pickup_goal.allow_gripper_support_collision = true; //allow gripper to touch table
      pickup_goal.target = processing_call.response.graspable_objects.at(0);
      pickup_goal.collision_object_name = processing_call.response.collision_object_names.at(0);
      pickup_goal.collision_support_surface_name = processing_call.response.collision_support_surface_name;
      pickup_goal.arm_name = "right_arm";
      geometry_msgs::Vector3Stamped direction;
      direction.header.stamp = ros::Time::now();
      direction.header.frame_id = "base_link";
      direction.vector.x = 0;
      direction.vector.y = 0;
      direction.vector.z = 1;
      pickup_goal.lift.direction = direction;
      pickup_goal.lift.desired_distance = 0.1;
      pickup_goal.lift.min_distance = 0.05;
      pickup_goal.use_reactive_lift = false;
      pickup_goal.use_reactive_execution = false;
      pickup_client.sendGoal(pickup_goal);
      while (!pickup_client.waitForResult(ros::Duration(10.0)))
      {
        ROS_INFO("Waiting for the pickup action...");
      }
      object_manipulation_msgs::PickupResult pickup_result = *(pickup_client.getResult());

//      ros::Duration(7.0).sleep();

//// TEMPLATE GRASPING CODE BEGIN: after grasp execution return result to grasp service ////
	PlanningFeedback feedback;
	feedback.request.action = PlanningFeedback::Request::DONT_UPGRADE_LIB;
	feedback.request.feedback = pickup_result;
	ROS_INFO("Calling template grasp planning feedback...");
	if (!grasp_planning_feedback_client.call(feedback))
	{
	  ROS_INFO("Feedback service failed");
	}
//// TEMPLATE GRASPING CODE END: after grasp execution return result to grasp service ////

      if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_ERROR("The pickup action has failed with result code %d",
            pickup_result.manipulation_result.value);
        continue;
      }

      //remember where we picked the object up from
      geometry_msgs::PoseStamped pickup_location;
      if (processing_call.response.graspable_objects.at(0).potential_models.size() != 0)
      {
        pickup_location = processing_call.response.graspable_objects.at(0).potential_models[0].pose;
      }
      else
      {
        pickup_location.header = processing_call.response.graspable_objects.at(0).cluster.header;
        pickup_location.pose.position.x = processing_call.response.graspable_objects.at(0).cluster.points[0].x;
        pickup_location.pose.position.y = processing_call.response.graspable_objects.at(0).cluster.points[0].y;
        pickup_location.pose.position.z = processing_call.response.graspable_objects.at(0).cluster.points[0].z;
        pickup_location.pose.orientation.w = 1;
      }

      //remember where to look at after execution of place action
      head_goal.target.point = pickup_location.pose.position;

      //place object
      geometry_msgs::PoseStamped place_location = pickup_location;
      place_location.header.stamp = ros::Time::now();
      place_location.pose.position.x += 0.0;
      ROS_INFO("Calling the place action");
      object_manipulation_msgs::PlaceGoal place_goal;
      place_goal.allow_gripper_support_collision = true;
      place_goal.place_locations.push_back(place_location);
      place_goal.collision_object_name = processing_call.response.collision_object_names.at(0);
      place_goal.collision_support_surface_name = processing_call.response.collision_support_surface_name;
      place_goal.grasp = pickup_result.grasp;
      place_goal.arm_name = "right_arm";
      place_goal.place_padding = 0.02;
      place_goal.desired_retreat_distance = 0.1;
      place_goal.min_retreat_distance = 0.05;
      direction.header.stamp = ros::Time::now();
      direction.header.frame_id = "base_link";
      direction.vector.x = 0;
      direction.vector.y = 0;
      direction.vector.z = -1;
      place_goal.approach.direction = direction;
      place_goal.approach.desired_distance = 0.1;
      place_goal.approach.min_distance = 0.05;
      place_goal.use_reactive_place = false;
      place_client.sendGoal(place_goal);

//      ros::Duration(10.0).sleep();

//      while (!place_client.waitForResult(ros::Duration(10.0)))
//      {
//        ROS_INFO("Waiting for the place action...");
//      }
//      object_manipulation_msgs::PlaceResult place_result = *(place_client.getResult());
//      if (place_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
//      {
//        ROS_ERROR("Place failed with error code %d",
//            place_result.manipulation_result.value);
//        continue;
//      }

      ROS_INFO("Finished pickup and place.");
    }
    else if (uinp == "zero")
    {
      mov_handl.goToZeroPose();
      ros::Duration(3.0).sleep();
      HeadMovementHandler h_mov;
      h_mov.lookAt("base_link", head_goal.target.point.x, head_goal.target.point.y, head_goal.target.point.z);
    }
  }
  return 0;
}
