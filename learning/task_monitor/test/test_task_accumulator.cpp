/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		test_task_accumulator.cpp

  \author	Peter Pastor
  \date		Jun 19, 2011

 *********************************************************************/

// system includes
#include <task_recorder2_msgs/Accumulate.h>

// local includes
#include <task_monitor/task_accumulator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestTaskAccumulator");
  ros::NodeHandle node_handle("~");

  TaskAccumulator task_accumulator(node_handle);
  ros::Duration(1.0).sleep();

  ros::spin();
  return 0;
}
