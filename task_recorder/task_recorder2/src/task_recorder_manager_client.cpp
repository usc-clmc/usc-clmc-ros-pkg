/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_recorder_manager_client.cpp

  \author	Peter Pastor
  \date		Jun 9, 2011

 *********************************************************************/

// system includes
#include <task_recorder2/StartStreaming.h>
#include <task_recorder2/StartRecording.h>
#include <task_recorder2/StopRecording.h>
#include <task_recorder2/InterruptRecording.h>
#include <task_recorder2/GetDataSample.h>

#include <task_recorder2_utilities/task_description_utilities.h>
#include <usc_utilities/services.h>

// local includes
#include <task_recorder2/task_recorder_manager_client.h>

namespace task_recorder2
{

TaskRecorderManagerClient::TaskRecorderManagerClient(bool block_until_services_are_available)
  : node_handle_(ros::NodeHandle()), is_online_(false)
{
  start_streaming_service_client_ = node_handle_.serviceClient<task_recorder2::StartStreaming> (std::string("/TaskRecorderManager/start_streaming"));
  start_recording_service_client_ = node_handle_.serviceClient<task_recorder2::StartRecording> (std::string("/TaskRecorderManager/start_recording"));
  stop_recording_service_client_ = node_handle_.serviceClient<task_recorder2::StopRecording> (std::string("/TaskRecorderManager/stop_recording"));
  interrupt_recording_service_client_ = node_handle_.serviceClient<task_recorder2::InterruptRecording> (std::string("/TaskRecorderManager/interrupt_recording"));
  get_data_sample_service_client_ = node_handle_.serviceClient<task_recorder2::GetDataSample> (std::string("/TaskRecorderManager/get_data_sample"));
  if(block_until_services_are_available)
  {
    waitForServices();
  }
}

void TaskRecorderManagerClient::waitForServices()
{
  usc_utilities::waitFor(start_streaming_service_client_);
  usc_utilities::waitFor(start_recording_service_client_);
  usc_utilities::waitFor(stop_recording_service_client_);
  usc_utilities::waitFor(interrupt_recording_service_client_);
  usc_utilities::waitFor(get_data_sample_service_client_);
  is_online_ = true;
}

bool TaskRecorderManagerClient::checkForServices()
{
  is_online_ = true;
  is_online_ = is_online_ && usc_utilities::isReady(start_streaming_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(start_recording_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(stop_recording_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(interrupt_recording_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(get_data_sample_service_client_);
  return is_online_;
}

bool TaskRecorderManagerClient::startStreaming()
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::StartStreaming::Request start_request;
  task_recorder2::StartStreaming::Response start_response;
  if(!start_streaming_service_client_.call(start_request, start_response))
  {
    ROS_ERROR("Problems when calling >%s<.", start_streaming_service_client_.getService().c_str());
    return false;
  }
  if(start_response.return_code != task_recorder2::StartStreaming::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", start_streaming_service_client_.getService().c_str(), start_response.info.c_str());
    return false;
  }
  if(!start_response.info.empty())
  {
    ROS_INFO_STREAM(start_response.info);
  }
  return true;
}

bool TaskRecorderManagerClient::startRecording(const std::string description, const int id, ros::Time& start_time )
{
  task_recorder2_msgs::Description task_description;
  task_description.id = id;
  task_description.description.assign(description);
  return startRecording(task_description, start_time);
}

bool TaskRecorderManagerClient::startRecording(const task_recorder2_msgs::Description& description, ros::Time& start_time)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::StartRecording::Request start_request;
  start_request.description = description;
  task_recorder2::StartRecording::Response start_response;
  if(!start_recording_service_client_.call(start_request, start_response))
  {
    ROS_ERROR("Problems when calling >%s<.", start_recording_service_client_.getService().c_str());
    return false;
  }
  if(start_response.return_code != task_recorder2::StartRecording::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", start_recording_service_client_.getService().c_str(), start_response.info.c_str());
    return false;
  }
  start_time = start_response.start_time;
  if(!start_response.info.empty())
  {
    ROS_INFO_STREAM(start_response.info);
  }
  return true;
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const ros::Time& end_time,
                                              const int num_samples,
                                              std::vector<task_recorder2_msgs::DataSample>& messages)
{
  std::vector<std::string> no_message_names;
  return stopRecording(start_time, end_time, num_samples, no_message_names, messages);
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const ros::Time& end_time,
                                              const int num_samples,
                                              const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::StopRecording::Request stop_request;
  task_recorder2::StopRecording::Response stop_response;
  stop_request.crop_start_time = start_time;
  stop_request.crop_end_time = end_time;
  stop_request.num_samples = num_samples;
  stop_request.message_names = message_names;
  if(!stop_recording_service_client_.call(stop_request, stop_response))
  {
    ROS_ERROR("Problems when calling >%s<.", stop_recording_service_client_.getService().c_str());
    return false;
  }
  if(stop_response.return_code != task_recorder2::StopRecording::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", stop_recording_service_client_.getService().c_str(), stop_response.info.c_str());
    return false;
  }
  messages = stop_response.filtered_and_cropped_messages;
  ROS_INFO_STREAM_COND(!stop_response.info.empty(), stop_response.info);
  return true;
}

bool TaskRecorderManagerClient::interruptRecording()
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::InterruptRecording::Request interrupt_request;
  task_recorder2::InterruptRecording::Response interrupt_response;
  if(!interrupt_recording_service_client_.call(interrupt_request, interrupt_response))
  {
    ROS_ERROR("Problems when calling >%s<.", interrupt_recording_service_client_.getService().c_str());
    return false;
  }
  if(interrupt_response.return_code != task_recorder2::InterruptRecording::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", interrupt_recording_service_client_.getService().c_str(),
              interrupt_response.info.c_str());
    return false;
  }
  ROS_INFO_STREAM_COND(!interrupt_response.info.empty(), interrupt_response.info);
  return true;
}

bool TaskRecorderManagerClient::getDataSample(const task_recorder2_msgs::Description& description,
                                              task_recorder2_msgs::DataSample& data_sample)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::GetDataSample::Request get_data_sample_request;
  get_data_sample_request.description = description;
  task_recorder2::GetDataSample::Response get_data_sample_response;
  if(!get_data_sample_service_client_.call(get_data_sample_request, get_data_sample_response))
  {
    ROS_ERROR("Problems when calling >%s<.", get_data_sample_service_client_.getService().c_str());
    return false;
  }
  if(get_data_sample_response.return_code != task_recorder2::GetDataSample::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", interrupt_recording_service_client_.getService().c_str(),
              get_data_sample_response.info.c_str());
    return false;
  }
  data_sample = get_data_sample_response.data_sample;
  ROS_INFO_STREAM_COND(!get_data_sample_response.info.empty(), get_data_sample_response.info);
  return true;
}

}
