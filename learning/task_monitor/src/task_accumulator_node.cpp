/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_accumulator_node.cpp

  \author	Peter Pastor
  \date		Jun 9, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <usc_utilities/assert.h>

// local includes
#include <task_monitor/task_accumulator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TaskAccumulator");
  ros::NodeHandle node_handle("~");

  task_monitor::TaskAccumulator task_accumulator(node_handle);

  ros::Duration(1.0).sleep();
  ros::spin();
  return 1;
}
