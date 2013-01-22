/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         user_demonstration_recorder.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <grasp_template_planning/SimpleLabel.h>
#include <grasp_template_planning/object_detection_listener.h>
#include <grasp_template/grasp_template_params.h>
#include <grasp_template_planning/grasp_planning_params.h>
#include <grasp_template_planning/demo_writer.h>

using namespace std;
using namespace grasp_template_planning;

static vector<double> fingerpositions;
static string spread_joint;
static string fpos1_joint;
static string fpos2_joint;
static string fpos3_joint;

void jointStateCallback(const sensor_msgs::JointState& msg)
{
  for (unsigned int i = 0; i < msg.name.size(); i++)
  {
    if (msg.name[i].compare(spread_joint) == 0)
    {
      fingerpositions[0] = msg.position[i];
    }
    else if (msg.name[i].compare(fpos1_joint) == 0)
    {
      fingerpositions[1] = msg.position[i];
    }
    else if (msg.name[i].compare(fpos2_joint) == 0)
    {
      fingerpositions[2] = msg.position[i];
    }
    else if (msg.name[i].compare(fpos3_joint) == 0)
    {
      fingerpositions[3] = msg.position[i];
    }
  }
}

int main(int argc, char** argv)
{
  if (argc < 1)
  {
    ROS_ERROR("Filename for this demonstration is missing. "
        "Please, call this program with argument filename:=my_filename");
    return -1;
  }
  ros::init(argc, argv, "user_demonstration_recorder");
  ros::NodeHandle n;

//  ROS_WARN_STREAM("ITS: " << ros::Time::now());
  char input = 'p';
  ROS_INFO("Note: Only demonstrations with the right arm are recorded.");

  ROS_INFO("Please, free the robot's view on the object "
      "and enter an arbitrary character followed by enter.");
  cin >> input;

  /* obtain object cluster and table pose */
  sensor_msgs::PointCloud2 cluster;
  geometry_msgs::PoseStamped table_pose;
  {
    ObjectDetectionListener object_detection;
    object_detection.connectToObjectDetector(n);
    if (!object_detection.fetchClusterFromObjectDetector())
    {
      ROS_ERROR("Did not get result from tabletop_object_detector!");
      return -1;
    }
    object_detection.getClusterPC2(cluster);
    table_pose = object_detection.getTableFrame();
  }

  /* obtain viewpoint */
  geometry_msgs::PoseStamped viewpoint_pose;
  {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    grasp_template::GraspTemplateParams params;
    viewpoint_pose.header.frame_id = params.frameBase();
    viewpoint_pose.header.stamp = ros::Time::now();
    try
    {
      if (!listener.waitForTransform(params.frameBase(),
            params.frameViewPoint(), ros::Time(0), ros::Duration(1)))
      {
        ROS_ERROR_STREAM("Waiting for transform, from " << params.frameBase()
            << " to " << params.frameViewPoint() << " timed out.");
        return -1;
      }
      else
      {
        listener.lookupTransform(params.frameBase(),
              params.frameViewPoint(), ros::Time(0), transform);
        viewpoint_pose.pose.position.x = transform.getOrigin().x();
        viewpoint_pose.pose.position.y = transform.getOrigin().y();
        viewpoint_pose.pose.position.z = transform.getOrigin().z();
        viewpoint_pose.pose.orientation.w = transform.getRotation().w();
        viewpoint_pose.pose.orientation.x = transform.getRotation().x();
        viewpoint_pose.pose.orientation.y = transform.getRotation().y();
        viewpoint_pose.pose.orientation.z = transform.getRotation().z();
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }

  ROS_INFO("Please, place the gripper at the object without displacing "
           "it and enter an arbitrary character followed by enter.");
  cin >> input;

  /* record gripper pose */
  geometry_msgs::PoseStamped gripper_pose;
  {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    grasp_template::GraspTemplateParams params;
    GraspPlanningParams plan_params;
    try
    {
      if (!listener.waitForTransform(params.frameBase(), plan_params.frameGripper(),
            ros::Time(0), ros::Duration(2)))
      {
          ROS_ERROR_STREAM("Waiting for transform, from " << params.frameBase()
              << " to " << plan_params.frameGripper() << " timed out.");
        return -1;
      }
      listener.lookupTransform(params.frameBase(), plan_params.frameGripper(),
            ros::Time(0), transform);
      gripper_pose.header.frame_id = params.frameBase();
      gripper_pose.header.stamp = ros::Time::now();
      gripper_pose.pose.position.x = transform.getOrigin().x();
      gripper_pose.pose.position.y = transform.getOrigin().y();
      gripper_pose.pose.position.z = transform.getOrigin().z();
      gripper_pose.pose.orientation.x = transform.getRotation().x();
      gripper_pose.pose.orientation.y = transform.getRotation().y();
      gripper_pose.pose.orientation.z = transform.getRotation().z();
      gripper_pose.pose.orientation.w = transform.getRotation().w();
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }

  fingerpositions.resize(4, numeric_limits<double>::max());
  string hand_architecture;
  ros::param::get("~hand_architecture", hand_architecture);
  if (hand_architecture == "armrobot")
  {
    ros::param::get("~fingerspread_joint", spread_joint);
    ros::param::get("~fingerpos1_joint", fpos1_joint);
    ros::param::get("~fingerpos2_joint", fpos2_joint);
    ros::param::get("~fingerpos3_joint", fpos3_joint);
    ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 100, jointStateCallback);

    ROS_INFO_STREAM("Obtaining finger joint-positions from " << spread_joint <<
         ", " << fpos1_joint << ", " << fpos2_joint << ", " << fpos3_joint << "...");
    while (ros::ok() && (fingerpositions[0] == numeric_limits<double>::max() ||
        fingerpositions[1] == numeric_limits<double>::max() || fingerpositions[2]
        == numeric_limits<double>::max() || fingerpositions[3] ==
        numeric_limits<double>::max()))
    {
      ros::spinOnce();
    }
    if ((fingerpositions[0] == numeric_limits<double>::max() || fingerpositions[1]
        == numeric_limits<double>::max() || fingerpositions[2] == numeric_limits<
        double>::max() || fingerpositions[3] == numeric_limits<double>::max()))
      ROS_ERROR("Could not record finger joint-positions.");
  }

  DemoWriter demo_writer(argv[1]);
  demo_writer.writeDemonstration(cluster, gripper_pose, table_pose, viewpoint_pose, fingerpositions);
  demo_writer.close();
  ROS_INFO("Done recording demonstration.");

  return 0;
}
