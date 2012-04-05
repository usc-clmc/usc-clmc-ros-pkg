/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_monitor.cpp

  \author	Peter Pastor
  \date		Jun 9, 2011

 *********************************************************************/

// system includes

// local includes
#include <task_monitor/task_monitor.h>

namespace task_monitor
{

TaskMonitor::TaskMonitor(ros::NodeHandle node_handle, const std::string topic_name)
  : node_handle_(node_handle)
{
  data_sample_subcriber_ = node_handle_.subscribe(topic_name, 1000, &TaskMonitor::monitorCallback, this);
  ROS_INFO("Monitoring on >%s<", data_sample_subcriber_.getTopic().c_str());

}

void TaskMonitor::monitorCallback(const task_recorder2_msgs::DataSampleConstPtr data_sample)
{

  if(!data_sample->data.empty())
  {
    ROS_INFO("Test: %f", data_sample->data[0]);
  }

}

}
