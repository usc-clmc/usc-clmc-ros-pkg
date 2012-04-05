/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    test_task_recorder_manager.cpp

 \author  Peter Pastor
 \date    Jun 05, 2011

 *********************************************************************/

// system includes
#include <sstream>
#include <ros/ros.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_utilities/task_recorder_specification_utilities.h>
#include <task_recorder2_msgs/TaskRecorderSpecification.h>

// local includes
#include <task_recorder2/task_recorder_manager.h>
#include <task_recorder2/task_recorder_manager_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestTaskRecorderManager");
  ros::NodeHandle node_handle("~");

  std::vector<task_recorder2_msgs::TaskRecorderSpecification> specifications;
  ROS_VERIFY(task_recorder2_utilities::readTaskRecorderSpecification(specifications));
  std::vector<std::string> variable_names;
  ROS_VERIFY(task_recorder2_utilities::getAllVariableNames(specifications, variable_names));

  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client;

  std::string description;
  if(!node_handle.getParam("description", description))
  {
    description.assign("test");
  }

  double duration;
  if(!node_handle.getParam("duration", duration))
  {
    duration = 3.0;
  }

  ros::Time start_time;
  ROS_VERIFY(task_recorder_manager_client.startRecording(description, 1, start_time));
  start_time = start_time + ros::Duration (0.2);

  ROS_INFO("Start recording...");
  ros::WallDuration(duration).sleep();
  ros::Time end_time = ros::Time::now() - ros::Duration (0.2);
  ROS_INFO("Stopped recording...");

  std::vector<task_recorder2_msgs::DataSample> filtered_and_cropped_messages;
  ROS_VERIFY(task_recorder_manager_client.stopRecording(start_time, end_time, 100, variable_names, filtered_and_cropped_messages));

  ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSample>::writeToBagFileWithTimeStamps(filtered_and_cropped_messages,
                                                                                                  "/TaskRecorderManager/data_sample",
                                                                                                  "/tmp/filtered_and_cropped_messages.bag", false));
  return 0;
}
