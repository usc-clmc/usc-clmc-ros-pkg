/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    test_task_recorder2_node.cpp

 \author  Peter Pastor
 \date    Jun 7, 2011

 *********************************************************************/

// system includes

// local includes
#include <task_recorder2/joint_states_recorder.h>
#include <task_recorder2/task_recorder_client.h>

using namespace task_recorder2;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestTaskRecorder");
  ros::NodeHandle node_handle("~");

  TaskRecorderClient<sensor_msgs::JointState> joint_states_recorder(node_handle);
  joint_states_recorder.initialize("/joint_states");

  ros::WallDuration(1.0).sleep();

  ROS_INFO("Starting to record...");
  joint_states_recorder.startRecording();

  ros::Time start_time = ros::Time::now() + ros::Duration(1.0);
  ros::WallDuration(3.0).sleep();
  ros::Time end_time = ros::Time::now() - ros::Duration (1.0);

  const int num_samples = 100;
  ROS_INFO("Stopping recording...");

  joint_states_recorder.stopRecording(start_time, end_time, num_samples);

  return 0;
}
