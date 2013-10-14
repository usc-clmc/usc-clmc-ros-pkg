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
#include <task_recorder2/StopStreaming.h>
#include <task_recorder2/StartRecording.h>
#include <task_recorder2/StopRecording.h>
#include <task_recorder2/InterruptRecording.h>
#include <task_recorder2/GetDataSample.h>
#include <task_recorder2/GetInfo.h>
#include <task_recorder2/AddDataSamples.h>
#include <task_recorder2/ReadDataSamples.h>

#include <task_recorder2_utilities/task_description_utilities.h>
#include <usc_utilities/services.h>

// local includes
#include <task_recorder2/task_recorder_manager_client.h>

namespace task_recorder2
{

TaskRecorderManagerClient::TaskRecorderManagerClient(bool block_until_services_are_available)
  : node_handle_(ros::NodeHandle()), is_online_(false), task_recorder_node_handle_("/TaskRecorderManager")
{
  start_streaming_service_client_ = node_handle_.serviceClient<task_recorder2::StartStreaming> (std::string("/TaskRecorderManager/start_streaming"));
  stop_streaming_service_client_ = node_handle_.serviceClient<task_recorder2::StopStreaming> (std::string("/TaskRecorderManager/stop_streaming"));
  start_recording_service_client_ = node_handle_.serviceClient<task_recorder2::StartRecording> (std::string("/TaskRecorderManager/start_recording"));
  stop_recording_service_client_ = node_handle_.serviceClient<task_recorder2::StopRecording> (std::string("/TaskRecorderManager/stop_recording"));
  interrupt_recording_service_client_ = node_handle_.serviceClient<task_recorder2::InterruptRecording> (std::string("/TaskRecorderManager/interrupt_recording"));
  get_data_sample_service_client_ = node_handle_.serviceClient<task_recorder2::GetDataSample> (std::string("/TaskRecorderManager/get_data_sample"));
  add_data_samples_service_client_ = node_handle_.serviceClient<task_recorder2::AddDataSamples> (std::string("/TaskRecorderManager/add_data_samples"));
  read_data_samples_service_client_ = node_handle_.serviceClient<task_recorder2::ReadDataSamples> (std::string("/TaskRecorderManager/read_data_samples"));
  get_info_service_client_ = node_handle_.serviceClient<task_recorder2::GetInfo> (std::string("/TaskRecorderManager/get_info"));
  if(block_until_services_are_available)
  {
    waitForServices();
  }
  description_marker_keys_.push_back("recording: ");
  description_marker_keys_.push_back("streaming: ");
  description_marker_values_.resize(description_marker_keys_.size(), "");

  const int PUBLISHER_BUFFER_SIZE = 1;
  status_publisher_ = task_recorder_node_handle_.advertise<visualization_msgs::Marker> ("visualization_marker", PUBLISHER_BUFFER_SIZE);
  setupDescriptionMarker();
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
  publishStreamingMarker("starting");
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::StartStreaming::Request start_request;
  task_recorder2::StartStreaming::Response start_response;
  if(!start_streaming_service_client_.call(start_request, start_response))
  {
    ROS_ERROR("Problems when calling >%s<.", start_streaming_service_client_.getService().c_str());
    publishDescriptionMarker(PROBLEM);
    publishStreamingMarker("failed");
    return false;
  }
  if(start_response.return_code != task_recorder2::StartStreaming::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", start_streaming_service_client_.getService().c_str(), start_response.info.c_str());
    publishDescriptionMarker(PROBLEM);
    publishStreamingMarker("failed");
    return false;
  }
  ROS_INFO_STREAM_COND(!start_response.info.empty(), start_response.info);
  publishStreamingMarker("started");
  return true;
}

bool TaskRecorderManagerClient::stopStreaming()
{
  publishStreamingMarker("stopping");
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::StopStreaming::Request stop_request;
  task_recorder2::StopStreaming::Response stop_response;
  if(!stop_streaming_service_client_.call(stop_request, stop_response))
  {
    ROS_ERROR("Problems when calling >%s<.", stop_streaming_service_client_.getService().c_str());
    publishDescriptionMarker(PROBLEM);
    publishStreamingMarker("failed");
    return false;
  }
  if(stop_response.return_code != task_recorder2::StopStreaming::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", stop_streaming_service_client_.getService().c_str(), stop_response.info.c_str());
    publishDescriptionMarker(PROBLEM);
    publishStreamingMarker("failed");
    return false;
  }
  ROS_INFO_STREAM_COND(!stop_response.info.empty(), stop_response.info);
  publishStreamingMarker("stopped");
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
  publishDescriptionMarker(description, STARTING_TO_RECORD);
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
    publishDescriptionMarker(description, PROBLEM);
    return false;
  }
  if(start_response.return_code != task_recorder2::StartRecording::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s", start_recording_service_client_.getService().c_str(), start_response.info.c_str());
    publishDescriptionMarker(description, PROBLEM);
    return false;
  }
  start_time = start_response.start_time;
  ROS_INFO_STREAM_COND(!start_response.info.empty(), start_response.info);
  publishDescriptionMarker(description, RECORDING);
  return true;
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const ros::Time& end_time,
                                              const int num_samples,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const bool stop_recording)
{
  std::vector<std::string> no_message_names;
  return stopRecording(start_time, end_time, num_samples, no_message_names, messages, stop_recording);
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const ros::Time& end_time,
                                              const int num_samples,
                                              const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const bool stop_recording)
{
  publishDescriptionMarker(STOPPING_TO_RECORD);
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
  stop_request.stop_recording = stop_recording;
  if(!stop_recording_service_client_.call(stop_request, stop_response))
  {
    ROS_ERROR("Problems when calling >%s<.", stop_recording_service_client_.getService().c_str());
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(stop_response.return_code != task_recorder2::StopRecording::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s (%i)", stop_recording_service_client_.getService().c_str(),
              stop_response.info.c_str(), stop_response.return_code);
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  messages = stop_response.filtered_and_cropped_messages;
  ROS_INFO_STREAM_COND(!stop_response.info.empty(), stop_response.info);
  publishDescriptionMarker(IDLE);
  return true;
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const ros::Time& end_time,
                                              const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const bool stop_recording)
{
  bool is_recording;
  ros::Time first;
  ros::Time last;
  double sampling_rate;
  if(!getInfo(is_recording, first, last, sampling_rate))
  {
    ROS_ERROR("Could not stop recording.");
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(!is_recording)
  {
    ROS_ERROR("Task recorders are not recording, cannot stop.");
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(start_time < first)
  {
    ROS_ERROR("Requested start time >%f< is invalid. First recorded sample has time stamp is >%f<.", start_time.toSec(), first.toSec());
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(end_time > last)
  {
    ROS_ERROR("Requested end time >%f< is invalid. Last recorded sample has time stamp is >%f<.", end_time.toSec(), last.toSec());
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  ros::Duration duration = end_time - start_time;
  const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
  ROS_INFO("Recorded >%.2f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
  return stopRecording(start_time, end_time, num_samples, message_names, messages, stop_recording);
}

bool TaskRecorderManagerClient::stopRecording(const ros::Time& start_time,
                                              const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const bool stop_recording)
{
  bool is_recording;
  ros::Time first;
  ros::Time last;
  double sampling_rate;
  if(!getInfo(is_recording, first, last, sampling_rate))
  {
    ROS_ERROR("Could not stop recording.");
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(!is_recording)
  {
    ROS_ERROR("Task recorders are not recording, cannot stop.");
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(start_time < first)
  {
    ROS_ERROR("Requested start time >%f< is invalid. First recorded sample has time stamp is >%f<.", start_time.toSec(), first.toSec());
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  ros::Duration duration = last - start_time;
  const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
  ROS_INFO("Recorded >%f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
  return stopRecording(start_time, last, num_samples, message_names, messages, stop_recording);
}

bool TaskRecorderManagerClient::stopRecording(const std::vector<std::string>& message_names,
                                              std::vector<task_recorder2_msgs::DataSample>& messages,
                                              const bool stop_recording)
{
  bool is_recording;
  ros::Time first;
  ros::Time last;
  double sampling_rate;
  if(!getInfo(is_recording, first, last, sampling_rate))
  {
    ROS_ERROR("Could not stop recording.");
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(!is_recording)
  {
    ROS_ERROR("Task recorders are not recording, cannot stop.");
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  ros::Duration duration = last - first;
  const int num_samples = static_cast<int>(duration.toSec() * sampling_rate);
  ROS_INFO("Recorded >%f< seconds and asking for >%i< samples.", duration.toSec(), num_samples);
  return stopRecording(first, last, num_samples, message_names, messages, stop_recording);
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
    publishDescriptionMarker(PROBLEM);
    return false;
  }
  if(interrupt_response.return_code != task_recorder2::InterruptRecording::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s (%i)", interrupt_recording_service_client_.getService().c_str(),
              interrupt_response.info.c_str(), interrupt_response.return_code);
    publishDescriptionMarker(PROBLEM);
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
    ROS_ERROR("Service >%s< was not successful: %s (%i)", get_data_sample_service_client_.getService().c_str(),
              get_data_sample_response.info.c_str(), get_data_sample_response.return_code);
    return false;
  }
  data_sample = get_data_sample_response.data_sample;
  ROS_INFO_STREAM_COND(!get_data_sample_response.info.empty(), get_data_sample_response.info);
  return true;
}

bool TaskRecorderManagerClient::addDataSamples(const task_recorder2_msgs::Description& description,
                                               const std::vector<task_recorder2_msgs::DataSample>& data_samples)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::AddDataSamples::Request add_data_samples_request;
  add_data_samples_request.description = description;
  add_data_samples_request.data_samples = data_samples;
  task_recorder2::AddDataSamples::Response add_data_samples_response;
  if(!add_data_samples_service_client_.call(add_data_samples_request, add_data_samples_response))
  {
    ROS_ERROR("Problems when calling >%s<.", add_data_samples_service_client_.getService().c_str());
    return false;
  }
  if(add_data_samples_response.return_code != task_recorder2::AddDataSamples::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s (%i)", add_data_samples_service_client_.getService().c_str(),
              add_data_samples_response.info.c_str(), add_data_samples_response.return_code);
    return false;
  }
  ROS_INFO_STREAM_COND(!add_data_samples_response.info.empty(), add_data_samples_response.info);
  return true;
}

bool TaskRecorderManagerClient::readDataSamples(const task_recorder2_msgs::Description& description,
                                                std::vector<task_recorder2_msgs::DataSample>& data_samples)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::ReadDataSamples::Request read_data_samples_request;
  read_data_samples_request.description = description;
  task_recorder2::ReadDataSamples::Response read_data_samples_response;
  if(!read_data_samples_service_client_.call(read_data_samples_request, read_data_samples_response))
  {
    ROS_ERROR("Problems when calling >%s<.", read_data_samples_service_client_.getService().c_str());
    return false;
  }
  if(read_data_samples_response.return_code != task_recorder2::ReadDataSamples::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s (%i)", read_data_samples_service_client_.getService().c_str(),
              read_data_samples_response.info.c_str(), read_data_samples_response.return_code);
    return false;
  }
  data_samples = read_data_samples_response.data_samples;
  ROS_INFO_STREAM_COND(!read_data_samples_response.info.empty(), read_data_samples_response.info);
  return true;
}

bool TaskRecorderManagerClient::getInfo(const task_recorder2_msgs::Description& description,
                                        std::string& abs_file_name)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::GetInfo::Request get_info_request;
  get_info_request.description = description;
  task_recorder2::GetInfo::Response get_info_response;
  if(!get_info_service_client_.call(get_info_request, get_info_response))
  {
    ROS_ERROR("Problems when calling >%s<.", get_info_service_client_.getService().c_str());
    return false;
  }
  if(get_info_response.return_code != task_recorder2::GetInfo::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s (%i)", get_info_service_client_.getService().c_str(),
              get_info_response.info.c_str(), get_info_response.return_code);
    return false;
  }
  abs_file_name.assign(get_info_response.file_name);
  return true;
}

bool TaskRecorderManagerClient::getInfo(bool &is_recording, ros::Time& first, ros::Time& last, double& sampling_rate)
{
  if(!servicesAreReady())
  {
    waitForServices();
  }
  task_recorder2::GetInfo::Request get_info_request;
  task_recorder2::GetInfo::Response get_info_response;
  if(!get_info_service_client_.call(get_info_request, get_info_response))
  {
    ROS_ERROR("Problems when calling >%s<.", get_info_service_client_.getService().c_str());
    return false;
  }
  if(get_info_response.return_code != task_recorder2::GetInfo::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< was not successful: %s (%i)", get_info_service_client_.getService().c_str(),
              get_info_response.info.c_str(), get_info_response.return_code);
    return false;
  }
  ROS_INFO_STREAM_COND(!get_info_response.info.empty(), get_info_response.info);
  is_recording = get_info_response.is_recording;
  first = get_info_response.first_recorded_time_stamp;
  last = get_info_response.last_recorded_time_stamp;
  sampling_rate = get_info_response.sampling_rate;
  return true;
}

void TaskRecorderManagerClient::setupDescriptionMarker()
{
  //visualization_msgs::Marker marker;
  description_marker_.header.frame_id = "/BASE";
  description_marker_.id = 1;
  description_marker_.ns = "description";
  description_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // std::vector<double> text_base_offset;
  // ROS_VERIFY(usc_utilities::read(node_handle_, "text_base_offset", text_base_offset));
  // description_marker_.pose.position.x = text_base_offset[0];
  // description_marker_.pose.position.y = text_base_offset[1];
  // description_marker_.pose.position.z = text_base_offset[2];
  description_marker_.pose.position.x = 0.8;
  description_marker_.pose.position.y = 0.0;
  description_marker_.pose.position.z = 2.0;
  description_marker_.pose.orientation.w = 1.0;
  description_marker_.pose.orientation.x = 0.0;
  description_marker_.pose.orientation.y = 0.0;
  description_marker_.pose.orientation.z = 0.0;
  description_marker_.scale.x = 1.0;
  description_marker_.scale.y = 1.0;
  description_marker_.scale.z = 0.08;
  description_marker_.color.r = 0.8;
  description_marker_.color.g = 0.8;
  description_marker_.color.b = 0.8;
  description_marker_.color.a = 0.8;

  description_marker_.text = "recording: \nstreaming:";
  publishDescriptionMarker(IDLE);
}

void TaskRecorderManagerClient::publishDescriptionMarker(const std::string& description, const int id, eRecorderStatus recorder_status)
{
  task_recorder2_msgs::Description tmp_description;
  tmp_description.description = description;
  tmp_description.id = id;
  publishDescriptionMarker(tmp_description, recorder_status);
}

void TaskRecorderManagerClient::publishDescriptionMarker(const std::string& description,
                                                         const int id,
                                                         bool publish)
{
  task_recorder2_msgs::Description tmp_description;
  tmp_description.description = description;
  tmp_description.id = id;
  publishDescriptionMarker(tmp_description, publish);
}

void TaskRecorderManagerClient::publishDescriptionMarker(const task_recorder2_msgs::Description& description,
                                                         eRecorderStatus recorder_status)
{
  publishDescriptionMarker(description, false);
  publishDescriptionMarker(recorder_status);
}

void TaskRecorderManagerClient::publishDescriptionMarker(const task_recorder2_msgs::Description& description,
                                                         bool publish)
{
  std::stringstream ss; ss << description.id;
  description_marker_values_[0] = description.description + "_" + ss.str();
  description_marker_.text.clear();
  for (unsigned int i = 0; i < description_marker_keys_.size(); ++i)
  {
    description_marker_.text.append(description_marker_keys_[i] + description_marker_values_[i] + "\n");
  }
  publishDescriptionMarker(publish);
}

void TaskRecorderManagerClient::publishDescriptionMarker(bool publish)
{
  if(publish)
  {
    description_marker_.header.stamp = ros::Time::now();
    status_publisher_.publish(description_marker_);
  }
}

void TaskRecorderManagerClient::publishDescriptionMarker(eRecorderStatus recorder_status, bool publish)
{
  switch (recorder_status)
  {
    case IDLE:
    {
      description_marker_.color.r = 0.8;
      description_marker_.color.g = 0.8;
      description_marker_.color.b = 0.8;
      description_marker_.color.a = 0.8;
      break;
    }
    case STARTING_TO_RECORD:
    {
      description_marker_.color.r = 0.8;
      description_marker_.color.g = 0.8;
      description_marker_.color.b = 0.0;
      description_marker_.color.a = 0.8;
      break;
    }
    case RECORDING:
    {
      description_marker_.color.r = 1.0;
      description_marker_.color.g = 0.0;
      description_marker_.color.b = 0.0;
      description_marker_.color.a = 0.8;
      break;
    }
    case STOPPING_TO_RECORD:
    {
      description_marker_.color.r = 0.8;
      description_marker_.color.g = 0.8;
      description_marker_.color.b = 0.0;
      description_marker_.color.a = 0.8;
      break;
    }
    case PROBLEM:
    {
      description_marker_.color.r = 1.0;
      description_marker_.color.g = 0.2;
      description_marker_.color.b = 1.0;
      description_marker_.color.a = 1.0;
      break;
    }
    default:
      ROS_ERROR("This should never happen.");
  }
  publishDescriptionMarker(publish);
}

void TaskRecorderManagerClient::publishStreamingMarker(const std::string& info, bool publish)
{
  description_marker_values_[1] = info;
  description_marker_.text.clear();
  for (unsigned int i = 0; i < description_marker_keys_.size(); ++i)
  {
    description_marker_.text.append(description_marker_keys_[i] + description_marker_values_[i] + "\n");
  }
  publishDescriptionMarker(publish);
}

}
