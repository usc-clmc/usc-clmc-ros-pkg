/*
 * grasp_object_test.cpp
 *
 *  Created on: Nov 16, 2010
 *      Author: kalakris
 */

#include <ros/ros.h>
#include <behavior_actions/GraspObjectAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<behavior_actions::GraspObjectAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_object_test");
  Client client("/Behaviors/GraspObject", true);
  client.waitForServer();

  behavior_actions::GraspObjectGoal goal;

  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult();
  behavior_actions::GraspObjectResult::ConstPtr result = client.getResult();
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED
      && result->result == result->SUCCEEDED)
    printf("It worked! (theoretically...)\n");
  else
    printf("It failed!\n");
  //printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
