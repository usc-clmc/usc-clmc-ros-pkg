/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_monitor_node.cpp

  \author	Peter Pastor
  \date		Jun 9, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <usc_utilities/assert.h>

// local includes
#include <task_monitor/task_monitor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TaskMonitor");
  ros::NodeHandle node_handle("~");

  task_monitor::TaskMonitor task_monitor(node_handle);
 // ROS_VERIFY(task_monitor.initialize(std::string("/joint_states")));

  ros::spin();
  return 1;
}
