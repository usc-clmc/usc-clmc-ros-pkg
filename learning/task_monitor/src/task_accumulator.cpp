/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_accumulator.cpp

  \author	Peter Pastor
  \date		Jun 9, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <task_monitor/task_accumulator.h>

namespace task_monitor
{

TaskAccumulator::TaskAccumulator(ros::NodeHandle node_handle)
  : accumulator_io_(node_handle)
{
  ROS_VERIFY(usc_utilities::read(node_handle, "topic_name", topic_name_));
  accumulator_service_ = accumulator_io_.node_handle_.advertiseService(std::string("accumulate"),
                                                                       &TaskAccumulator::accumulate, this);
  ROS_VERIFY(accumulator_io_.initialize());
}

bool TaskAccumulator::accumulate(task_recorder2_msgs::Accumulate::Request& request,
                                 task_recorder2_msgs::Accumulate::Response& response)
{
  // ROS_INFO("Accumulating >%i< trials...", request.description.trial+1);
  if(!accumulator_io_.accumulate(request.description))
  {
    response.info.append(std::string("Failed to accumulate data samples."));
    response.return_code = task_recorder2_msgs::Accumulate::Response::SERVICE_CALL_FAILED;
    return true;
  }
  response.info.append(std::string("Accumulated >") + usc_utilities::getString(request.description.trial+1) + std::string("< data traces."));
  response.return_code = task_recorder2_msgs::Accumulate::Response::SERVICE_CALL_SUCCESSFUL;
  return true;
}

}
