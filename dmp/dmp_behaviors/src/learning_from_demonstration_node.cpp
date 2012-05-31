/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   learning_from_demonstration_node.cpp

  \author Peter Pastor
  \date   Jan 12, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <robot_info/robot_info_init.h>

// local includes
#include <dmp_behaviors/learning_from_demonstration.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  robot_info::init();
  ros::NodeHandle node_handle("~");
  std::string action_name = "learningFromDemonstration";
  ROS_INFO("Setting up >%s< action.", action_name.c_str());
  dmp_behaviors::LearningFromDemonstration learning_from_demonstration(node_handle, action_name);
  ROS_INFO("Starting >%s< action.", action_name.c_str());
  learning_from_demonstration.start();
  ros::spin();
}
