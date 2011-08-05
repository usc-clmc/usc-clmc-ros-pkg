/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_recorder2_node.cpp

  \author	Peter Pastor
  \date		Jun 7, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <usc_utilities/assert.h>

// local includes
#include <task_recorder2/joint_states_recorder.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TaskRecorder");
  ros::NodeHandle node_handle("~");

  task_recorder2::JointStatesRecorder joint_state_recorder(node_handle);
  ROS_VERIFY(joint_state_recorder.initialize());

  ros::spin();
  return 1;
}
