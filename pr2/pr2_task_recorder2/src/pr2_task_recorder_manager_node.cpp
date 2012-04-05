/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_task_recorder_manager_node.cpp

  \author	Peter Pastor
  \date		Jun 7, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>

// local includes
#include <pr2_task_recorder2/pr2_task_recorder_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TaskRecorderManager");
  ros::NodeHandle node_handle("~");

  pr2_task_recorder2::PR2TaskRecorderManager pr2_task_recorder_manager(node_handle);
  if(!pr2_task_recorder_manager.initialize())
  {
    ROS_ERROR("Could not initialize pr2 task recorder manager.");
    return -1;
  }

  ros::Duration(1.0).sleep();
  ros::spin();
  return 0;
}
