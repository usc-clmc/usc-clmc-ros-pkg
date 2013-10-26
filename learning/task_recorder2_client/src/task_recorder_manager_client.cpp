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
#include <task_recorder2_srvs/StartStreaming.h>
#include <task_recorder2_srvs/StopStreaming.h>
#include <task_recorder2_srvs/StartRecording.h>
#include <task_recorder2_srvs/StopRecording.h>
#include <task_recorder2_srvs/InterruptRecording.h>
#include <task_recorder2_srvs/GetDataSample.h>
#include <task_recorder2_srvs/GetInfo.h>
#include <task_recorder2_srvs/AddDataSamples.h>
#include <task_recorder2_srvs/ReadDataSamples.h>

#include <blackboard/BlackBoardEntry.h>

#include <task_recorder2_utilities/task_description_utilities.h>
#include <usc_utilities/services.h>

// local includes
#include <task_recorder2_client/task_recorder_manager_client.h>

namespace task_recorder2_client
{

TaskRecorderManagerClient::TaskRecorderManagerClient(bool block_until_services_are_available)
  : node_handle_(ros::NodeHandle()), is_online_(false)
{
  start_streaming_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::StartStreaming> (std::string("/TaskRecorderManager/startStreaming"));
  stop_streaming_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::StopStreaming> (std::string("/TaskRecorderManager/stopStreaming"));
  start_recording_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::StartRecording> (std::string("/TaskRecorderManager/startRecording"));
  stop_recording_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::StopRecording> (std::string("/TaskRecorderManager/stopRecording"));
  interrupt_recording_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::InterruptRecording> (std::string("/TaskRecorderManager/interruptRecording"));
  get_data_sample_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::GetDataSample> (std::string("/TaskRecorderManager/getDataSample"));
  add_data_samples_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::AddDataSamples> (std::string("/TaskRecorderManager/addDataSamples"));
  read_data_samples_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::ReadDataSamples> (std::string("/TaskRecorderManager/readDataSamples"));
  get_info_service_client_ = node_handle_.serviceClient<task_recorder2_srvs::GetInfo> (std::string("/TaskRecorderManager/getInfo"));
  if(block_until_services_are_available)
  {
    waitForServices();
  }
}

void TaskRecorderManagerClient::waitForServices()
{
  usc_utilities::waitFor(start_streaming_service_client_);
  usc_utilities::waitFor(stop_streaming_service_client_);
  usc_utilities::waitFor(start_recording_service_client_);
  usc_utilities::waitFor(stop_recording_service_client_);
  usc_utilities::waitFor(interrupt_recording_service_client_);
  usc_utilities::waitFor(get_data_sample_service_client_);
  usc_utilities::waitFor(add_data_samples_service_client_);
  usc_utilities::waitFor(read_data_samples_service_client_);
  usc_utilities::waitFor(get_info_service_client_);
  is_online_ = true;
}

bool TaskRecorderManagerClient::checkForServices()
{
  is_online_ = true;
  is_online_ = is_online_ && usc_utilities::isReady(start_streaming_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(stop_streaming_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(start_recording_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(stop_recording_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(interrupt_recording_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(get_data_sample_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(add_data_samples_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(read_data_samples_service_client_);
  is_online_ = is_online_ && usc_utilities::isReady(get_info_service_client_);
  return is_online_;
}

bool TaskRecorderManagerClient::startStreaming()
{
  blackboard_client_.startStreaming();
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::StartStreaming::Request request;
  task_recorder2_srvs::StartStreaming::Response response;
  if(!start_streaming_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s<.", start_streaming_service_client_.getService().c_str());
    blackboard_client_.streamingFailed();
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              start_streaming_service_client_.getService().c_str(), response.info.c_str());
    blackboard_client_.streamingFailed();
    return false;
  }
  ROS_DEBUG_STREAM_COND(!response.info.empty(), response.info);
  blackboard_client_.streaming();
  return true;
}

bool TaskRecorderManagerClient::stopStreaming()
{
  blackboard_client_.stopStreaming();
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::StopStreaming::Request request;
  task_recorder2_srvs::StopStreaming::Response response;
  if(!stop_streaming_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s<.", stop_streaming_service_client_.getService().c_str());
    blackboard_client_.streamingFailed();
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              stop_streaming_service_client_.getService().c_str(), response.info.c_str());
    blackboard_client_.streamingFailed();
    return false;
  }
  ROS_DEBUG_STREAM_COND(!response.info.empty(), response.info);
  blackboard_client_.streamingStopped();
  return true;
}

bool TaskRecorderManagerClient::startRecording(const std::string description,
                                               const int id,
                                               ros::Time& start_time)
{
  task_recorder2_msgs::Description task_description;
  task_description.id = id;
  task_description.description.assign(description);
  return startRecording(task_description, start_time);
}

bool TaskRecorderManagerClient::startRecording(const task_recorder2_msgs::Description& description,
                                               ros::Time& start_time)
{
  ROS_DEBUG("Start recording description >%s< with id >%i<.", description.description.c_str(), description.id);
  blackboard_client_.startRecording();
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::StartRecording::Request request;
  request.description = description;
  task_recorder2_srvs::StartRecording::Response response;
  if(!start_recording_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s<.", start_recording_service_client_.getService().c_str());
    blackboard_client_.recordingFailed();
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              start_recording_service_client_.getService().c_str(), response.info.c_str());
    blackboard_client_.recordingFailed();
    return false;
  }
  start_time = response.start_time;
  ROS_DEBUG_STREAM_COND(!response.info.empty(), response.info);
  blackboard_client_.recording();
  blackboard_client_.setupError(task_recorder2_utilities::getFileName(description));
  return true;
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const ros::Time& end_time,
                                              const int num_samples,
                                              const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const task_recorder2_msgs::Description& description,
                                              const bool stop_recording)
{
  blackboard_client_.stopRecording();
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::StopRecording::Request request;
  task_recorder2_srvs::StopRecording::Response response;
  request.crop_start_time = start_time;
  request.crop_end_time = end_time;
  request.num_samples = num_samples;
  request.message_names = message_names;
  request.stop_recording = stop_recording;
  request.description = description;
  if(!stop_recording_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s< : %s",
              stop_recording_service_client_.getService().c_str(), response.info.c_str());
    blackboard_client_.recordingFailed();
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              stop_recording_service_client_.getService().c_str(), response.info.c_str());
    blackboard_client_.recordingFailed();
    return false;
  }
  messages = response.filtered_and_cropped_messages;
  ROS_DEBUG_STREAM_COND(!response.info.empty(), response.info);
  blackboard_client_.recordingStopped();
  blackboard_client_.info(blackboard::BlackBoardEntry::SETUP_KEY);
  return true;
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const ros::Time& end_time,
                                              const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const task_recorder2_msgs::Description& description,
                                              const bool stop_recording)
{
  bool is_recording;
  ros::Time first;
  ros::Time last;
  double sampling_rate;
  if (!getInfo(is_recording, first, last, sampling_rate))
  {
    ROS_ERROR("Could not get info from task recorder manager. Could not stop recording.");
    blackboard_client_.recordingFailed();
    return false;
  }
  if (!stoppable(is_recording, first, last))
  {
    return false;
  }
  if(start_time < first)
  {
    ROS_ERROR("Requested start time >%f< is invalid. First recorded sample has time stamp >%f<.",
              start_time.toSec(), first.toSec());
    blackboard_client_.recordingFailed();
    return false;
  }
  if(end_time > last)
  {
    ROS_ERROR("Requested end time >%f< is invalid. Last recorded sample has time stamp >%f<.",
              end_time.toSec(), last.toSec());
    blackboard_client_.recordingFailed();
    return false;
  }
  ros::Duration duration = end_time - start_time;
  const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
  ROS_INFO("Recorded >%.2f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
  return stopRecording(start_time, end_time, num_samples, message_names, messages, description, stop_recording);
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const task_recorder2_msgs::Description& description,
                                              const bool stop_recording)
{
  bool is_recording;
  ros::Time first;
  ros::Time last;
  double sampling_rate;
  if (!getInfo(is_recording, first, last, sampling_rate))
  {
    ROS_ERROR("Could not get info from task recorder manager. Could not stop recording.");
    blackboard_client_.recordingFailed();
    return false;
  }
  if (!stoppable(is_recording, first, last))
  {
    return false;
  }
  if(start_time < first)
  {
    ROS_ERROR("Requested start time >%f< is invalid. First recorded sample has time stamp >%f<.",
              start_time.toSec(), first.toSec());
    blackboard_client_.recordingFailed();
    return false;
  }
  ros::Duration duration = last - start_time;
  const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
  ROS_INFO("Recorded >%.2f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
  return stopRecording(start_time, last, num_samples, message_names, messages, description, stop_recording);
}

bool TaskRecorderManagerClient::stopRecording(const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const task_recorder2_msgs::Description& description,
                                              const bool stop_recording)
{
  bool is_recording;
  ros::Time first;
  ros::Time last;
  double sampling_rate;
  if (!getInfo(is_recording, first, last, sampling_rate))
  {
    ROS_ERROR("Could not get info from task recorder manager. Could not stop recording.");
    blackboard_client_.recordingFailed();
    return false;
  }
  if (!stoppable(is_recording, first, last))
  {
    return false;
  }
  ros::Duration duration = last - first;
  const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
  ROS_INFO("Recorded >%.2f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
  return stopRecording(first, last, num_samples, message_names, messages, description, stop_recording);
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& end_time,
                                              const task_recorder2_msgs::Description& description,
                                              const bool stop_recording)
{
  blackboard_client_.stopRecording();
  bool is_recording;
  ros::Time first;
  ros::Time last;
  double sampling_rate;
  if (!getInfo(is_recording, first, last, sampling_rate))
  {
    ROS_ERROR("Could not get info from task recorder manager. Could not stop recording.");
    blackboard_client_.recordingFailed();
    return false;
  }
  if (!stoppable(is_recording, first, last))
  {
    return false;
  }
  if (end_time > last)
  {
    ROS_ERROR("Requested end time >%f< is invalid. Last recorded sample has time stamp >%f<.", end_time.toSec(), last.toSec());
    blackboard_client_.recordingFailed();
    return false;
  }
  ros::Duration duration = end_time - first;
  const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
  ROS_INFO("Recorded >%.2f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
  const std::vector<std::string> message_names;
  std::vector<task_recorder2_msgs::DataSample> messages;
  return stopRecording(first, end_time, num_samples, message_names, messages, description, stop_recording);
}

bool TaskRecorderManagerClient::stoppable(const bool is_recording,
                                      const ros::Time& first,
                                      const ros::Time& last)
{
  if(!is_recording && first == last)
  {
    ROS_ERROR("Recorders are not recording. Cannot stop.");
    blackboard_client_.recordingFailed();
    return false;
  }
  return true;
}

bool TaskRecorderManagerClient::interruptRecording(const bool recording)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::InterruptRecording::Request request;
  task_recorder2_srvs::InterruptRecording::Response response;
  request.recording = recording;
  if (request.recording)
    blackboard_client_.continueRecording();
  else
    blackboard_client_.interruptRecording();
  if(!interrupt_recording_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s< : %s",
              interrupt_recording_service_client_.getService().c_str(), response.info.c_str());
    blackboard_client_.recordingFailed();
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              interrupt_recording_service_client_.getService().c_str(), response.info.c_str());
    blackboard_client_.recordingFailed();
    return false;
  }
  if (request.recording)
    blackboard_client_.recordingContinued();
  else
    blackboard_client_.recordingInterrupted();
  ROS_DEBUG_STREAM_COND(!response.info.empty(), response.info);
  return true;
}

bool TaskRecorderManagerClient::getDataSample(const task_recorder2_msgs::Description& description,
                                              task_recorder2_msgs::DataSample& data_sample)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::GetDataSample::Request request;
  request.description = description;
  task_recorder2_srvs::GetDataSample::Response response;
  if(!get_data_sample_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s< : %s",
              get_data_sample_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              get_data_sample_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  data_sample = response.data_sample;
  ROS_INFO_STREAM_COND(!response.info.empty(), response.info);
  return true;
}

bool TaskRecorderManagerClient::addDataSamples(const task_recorder2_msgs::Description& description,
                                               const std::vector<task_recorder2_msgs::DataSample>& data_samples)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::AddDataSamples::Request request;
  request.description = description;
  request.data_samples = data_samples;
  task_recorder2_srvs::AddDataSamples::Response response;
  if(!add_data_samples_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s< : %s",
              add_data_samples_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              add_data_samples_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  ROS_INFO_STREAM_COND(!response.info.empty(), response.info);
  return true;
}

bool TaskRecorderManagerClient::readDataSamples(const task_recorder2_msgs::Description& description,
                                                std::vector<task_recorder2_msgs::DataSample>& data_samples)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::ReadDataSamples::Request request;
  request.description = description;
  task_recorder2_srvs::ReadDataSamples::Response response;
  if(!read_data_samples_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s< : %s",
              read_data_samples_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              read_data_samples_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  data_samples = response.data_samples;
  ROS_INFO_STREAM_COND(!response.info.empty(), response.info);
  return true;
}

bool TaskRecorderManagerClient::getInfo(const task_recorder2_msgs::Description& description,
                                        std::string& abs_file_name)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::GetInfo::Request request;
  request.description = description;
  task_recorder2_srvs::GetInfo::Response response;
  if(!get_info_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s< : %s",
              get_info_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              get_info_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  abs_file_name.assign(response.file_name);
  return true;
}

bool TaskRecorderManagerClient::getInfo(bool &is_recording, ros::Time& first, ros::Time& last, double& sampling_rate)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2_srvs::GetInfo::Request request;
  task_recorder2_srvs::GetInfo::Response response;
  if(!get_info_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when calling >%s< : %s",
              get_info_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  if(response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful : %s",
              get_info_service_client_.getService().c_str(), response.info.c_str());
    return false;
  }
  ROS_INFO_STREAM_COND(!response.info.empty(), response.info);
  is_recording = response.is_recording;
  first = response.first_recorded_time_stamp;
  last = response.last_recorded_time_stamp;
  sampling_rate = response.sampling_rate;
  return true;
}

}
