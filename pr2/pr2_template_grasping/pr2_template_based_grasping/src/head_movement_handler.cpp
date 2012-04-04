/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         head_movment_handler.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <pr2_template_based_grasping/head_movement_handler.h>

using namespace std;

namespace pr2_template_based_grasping
{

HeadMovementHandler::HeadMovementHandler()
{
  //Initialize the client for the Action interface to the head controller
  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

  //wait for head controller action server to come up
  while (!point_head_client_->waitForServer(ros::Duration(5.0)))
  {
    ROS_DEBUG("pr2_template_based_grasping::HeadMovementHandler: Waiting for the point_head_action server to come up");
  }
}

HeadMovementHandler::~HeadMovementHandler()
{
  delete point_head_client_;
}

void HeadMovementHandler::lookAt(string frame_id, double x, double y, double z)
{
  //the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;

  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x;
  point.point.y = y;
  point.point.z = z;
  goal.target = point;

  //we are pointing the camera frame
  //(pointing_axis defaults to X-axis)
  goal.pointing_frame = "high_def_frame";

  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.5);

  //and go no faster than 1 rad/s
  goal.max_velocity = 1.0;

  //send the goal
  point_head_client_->cancelGoalsAtAndBeforeTime(ros::Time::now());
  point_head_client_->sendGoal(goal);

  //wait for it to get there (abort after 2 secs to prevent getting stuck)
  point_head_client_->waitForResult(ros::Duration(2));
}
} //namespace
