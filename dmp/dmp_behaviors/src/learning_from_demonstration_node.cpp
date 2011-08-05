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
  dmp_behaviors::LearningFromDemonstration learning_from_demonstration(node_handle, action_name);
	learning_from_demonstration.start();
  ros::spin();
  return 0;
}
