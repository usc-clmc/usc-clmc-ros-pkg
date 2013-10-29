/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_recorder_manager.cpp

  \author	Peter Pastor
  \date		Jun 6, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_msgs/Notification.h>
#include <task_recorder2_utilities/data_sample_utilities.h>

#include <omp.h>

// local includes
#include <task_recorder2/task_recorder_manager.h>

namespace task_recorder2
{

TaskRecorderManager::TaskRecorderManager(ros::NodeHandle node_handle) :
    recorder_io_(node_handle), sampling_rate_(-1.0), counter_(0)
{
  ROS_DEBUG("Creating task recorder manager in namespace >%s<.", node_handle.getNamespace().c_str());
  ROS_VERIFY(recorder_io_.initialize(recorder_io_.node_handle_.getNamespace() + std::string("/data_samples")));
}

bool TaskRecorderManager::initialize()
{
  if (!read(task_recorders_))
  {
    ROS_ERROR("Problem initializing task recorder manager.");
    return false;
  }
  ROS_INFO("Initialized TaskRecorderManager with >%i< recorders.", (int)task_recorders_.size());
  if (task_recorders_.empty())
  {
    ROS_ERROR("No task recorders created. Cannot initialize task recorder manager.");
    return false;
  }

  // one thread for each recorder, hopefully that make sense
  async_spinner_.reset(new ros::AsyncSpinner(1 + task_recorders_.size()));

  data_samples_.resize(task_recorders_.size());
  start_streaming_requests_.resize(task_recorders_.size());
  start_streaming_responses_.resize(task_recorders_.size());
  stop_streaming_requests_.resize(task_recorders_.size());
  stop_streaming_responses_.resize(task_recorders_.size());
  start_recording_requests_.resize(task_recorders_.size());
  start_recording_responses_.resize(task_recorders_.size());
  stop_recording_requests_.resize(task_recorders_.size());
  stop_recording_responses_.resize(task_recorders_.size());
  interrupt_recording_requests_.resize(task_recorders_.size());
  interrupt_recording_responses_.resize(task_recorders_.size());

  ROS_VERIFY(usc_utilities::read(recorder_io_.node_handle_, "sampling_rate", sampling_rate_));
  if (!(sampling_rate_ > 0.0))
  {
    ROS_ERROR("Invalid sampling rate read >%f<. Cannot initialize task recorder manager.", sampling_rate_);
    return false;
  }
  const double UPDATE_TIMER_PERIOD = static_cast<double>(1.0) / sampling_rate_;
  timer_ = recorder_io_.node_handle_.createTimer(ros::Duration(UPDATE_TIMER_PERIOD), &TaskRecorderManager::timerCB, this);

  const int DATA_SAMPLE_PUBLISHER_BUFFER_SIZE = 1;
  data_sample_publisher_ = recorder_io_.node_handle_.advertise<task_recorder2_msgs::DataSample>("data_samples", DATA_SAMPLE_PUBLISHER_BUFFER_SIZE);
  stop_recording_publisher_ = recorder_io_.node_handle_.advertise<task_recorder2_msgs::Notification>("notification", 1);

  start_recording_service_server_ = recorder_io_.node_handle_.advertiseService("startRecording", &TaskRecorderManager::startRecording, this);
  stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService("stopRecording", &TaskRecorderManager::stopRecording, this);
  start_streaming_service_server_ = recorder_io_.node_handle_.advertiseService("startStreaming", &TaskRecorderManager::startStreaming, this);
  stop_streaming_service_server_ = recorder_io_.node_handle_.advertiseService("stopStreaming", &TaskRecorderManager::stopStreaming, this);
  interrupt_recording_service_server_ = recorder_io_.node_handle_.advertiseService("interruptRecording", &TaskRecorderManager::interruptRecording, this);
  get_info_service_server_ = recorder_io_.node_handle_.advertiseService("getInfo", &TaskRecorderManager::getInfo, this);
  get_data_sample_service_server_ = recorder_io_.node_handle_.advertiseService("getDataSample", &TaskRecorderManager::getDataSample, this);
  add_data_samples_service_server_ = recorder_io_.node_handle_.advertiseService("addDataSamples", &TaskRecorderManager::addDataSamples, this);
  read_data_samples_service_server_ = recorder_io_.node_handle_.advertiseService("readDataSamples", &TaskRecorderManager::readDataSamples, this);

  last_combined_data_sample_.names.clear();
  last_combined_data_sample_.data.clear();
  updateInfo();

  omp_set_num_threads(task_recorders_.size());

  return true;
}

TaskRecorderManager::~TaskRecorderManager()
{
  async_spinner_->stop();
}

void TaskRecorderManager::run()
{
  async_spinner_->start();
  ros::spin();
}

unsigned int TaskRecorderManager::getNumberOfTaskRecorders() const
{
  return task_recorders_.size();
}

bool TaskRecorderManager::startStreaming(task_recorder2_srvs::StartStreaming::Request& request,
                                         task_recorder2_srvs::StartStreaming::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    start_streaming_requests_[i] = request;
  }

#pragma omp parallel for
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    ROS_VERIFY(task_recorders_[i]->startStreaming(start_streaming_requests_[i], start_streaming_responses_[i]));
  }

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    // response.info.append(start_streaming_responses_[i].info);
    if (start_streaming_responses_[i].return_code != start_streaming_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = start_streaming_responses_[i].return_code;
      ROS_ERROR_STREAM_COND(!start_streaming_responses_[i].info.empty(), start_streaming_responses_[i].info);
    }
  }
  return true;
}

bool TaskRecorderManager::stopStreaming(task_recorder2_srvs::StopStreaming::Request& request,
                                        task_recorder2_srvs::StopStreaming::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    stop_streaming_requests_[i] = request;
  }

#pragma omp parallel for
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    ROS_VERIFY(task_recorders_[i]->stopStreaming(stop_streaming_requests_[i], stop_streaming_responses_[i]));
  }

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    // response.info.append(stop_streaming_responses_[i].info);
    if (stop_streaming_responses_[i].return_code != stop_streaming_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = stop_streaming_responses_[i].return_code;
      ROS_ERROR_STREAM_COND(!stop_streaming_responses_[i].info.empty(), stop_streaming_responses_[i].info);
    }
  }
  return true;
}

bool TaskRecorderManager::startRecording(task_recorder2_srvs::StartRecording::Request& request,
                                         task_recorder2_srvs::StartRecording::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  resetInterruptHandler();
  recorder_io_.setDescription(request.description);

  response.start_time = ros::TIME_MAX;
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    start_recording_requests_[i] = request;
  }

#pragma omp parallel for
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    ROS_VERIFY(task_recorders_[i]->startRecording(start_recording_requests_[i], start_recording_responses_[i]));
  }

  response.start_time = ros::TIME_MAX;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    response.info.append(start_recording_responses_[i].info);
    if (start_recording_responses_[i].return_code != start_recording_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = start_recording_responses_[i].return_code;
      ROS_ERROR_STREAM_COND(!start_recording_responses_[i].info.empty(), start_recording_responses_[i].info);
      blackboard_client_.setLoggingUnknown();
    }
    if (start_recording_responses_[i].start_time > response.start_time)
    {
      response.start_time = start_recording_responses_[i].start_time;
    }
  }

  if (response.return_code == response.SERVICE_CALL_SUCCESSFUL)
  {
    response.info.append("Started to record at >" + boost::lexical_cast<std::string>(response.start_time.toSec()) + "<.");
    blackboard_client_.setLogging(true);
  }
  return true;
}

bool TaskRecorderManager::stopRecording(task_recorder2_srvs::StopRecording::Request& request,
                                        task_recorder2_srvs::StopRecording::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  if (request.num_samples == 0)
  {
    response.info = "Number of samples is zero. Cannot stop recording.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }
  getInterrupts(request.interrupt_start_stamps, request.interrupt_durations);
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    stop_recording_requests_[i] = request;
  }

#pragma omp parallel for
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    // ensure that recorders return filtered and cropped messages
    stop_recording_requests_[i].return_filtered_and_cropped_messages = true;
    stop_recording_requests_[i].message_names.clear();
    ROS_VERIFY(task_recorders_[i]->stopRecording(stop_recording_requests_[i], stop_recording_responses_[i]));
  }

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    response.info.append(stop_recording_responses_[i].info);
    if (stop_recording_responses_[i].return_code != stop_recording_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = stop_recording_responses_[i].return_code;
      ROS_ERROR_STREAM_COND(!stop_streaming_responses_[i].info.empty(), stop_streaming_responses_[i].info);
    }
  }
  if (response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    blackboard_client_.setLoggingUnknown();
    return true;
  }
  blackboard_client_.setLogging(!request.stop_recording);

  // update description and create directories if needed
  response.description = request.description;
  if (response.description.description.empty())
  {
    response.description = recorder_io_.getDescription();
  }
  recorder_io_.setDescription(response.description);
  if (!recorder_io_.createDirectories())
  {
    response.info = "Problem creating directories for >" + recorder_io_.topic_name_ + "<. Cannot stop recording.\n";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  unsigned int num_messages = 0;
  std::vector<std::string> all_variable_names;
  unsigned int num_messages_of_first_recorder = stop_recording_responses_[0].filtered_and_cropped_messages.size();
  if (num_messages_of_first_recorder == 0)
  {
    response.info = "First recorder is empty. Cannot stop recording.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    // error checking
    num_messages = stop_recording_responses_[i].filtered_and_cropped_messages.size();
    if (num_messages != num_messages_of_first_recorder)
    {
      response.info = "Number of messages differ among recorder " + boost::lexical_cast<std::string>(i) + " >"
                           + boost::lexical_cast<std::string>(num_messages) + "< and the first recorder >"
                           + boost::lexical_cast<std::string>(num_messages_of_first_recorder) + "<.";
      ROS_ERROR_STREAM(response.info);
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
    }
    all_variable_names.insert(all_variable_names.end(),
                              stop_recording_responses_[i].filtered_and_cropped_messages[0].names.begin(),
                              stop_recording_responses_[i].filtered_and_cropped_messages[0].names.end());
  }

  // clear all data samples
  task_recorder2_msgs::DataSample data_sample;
  data_sample.names = all_variable_names;
  data_sample.data.resize(data_sample.names.size(), 0.0);
  recorder_io_.messages_.resize(num_messages, data_sample);
  for (unsigned int i = 0; i < num_messages; ++i)
  {
    recorder_io_.messages_[i].header.seq = i;
    recorder_io_.messages_[i].header.stamp = stop_recording_responses_[0].filtered_and_cropped_messages[i].header.stamp;
    unsigned int index = 0;
    for (unsigned int j = 0; j < task_recorders_.size(); ++j)
    {
      for (unsigned int n = 0; n < stop_recording_responses_[j].filtered_and_cropped_messages[i].data.size(); ++n)
      {
        recorder_io_.messages_[i].data[index++] = stop_recording_responses_[j].filtered_and_cropped_messages[i].data[n];
      }
    }
  }

  if (request.return_filtered_and_cropped_messages)
  {
    if(request.message_names.empty())
    {
      // extract all data samples
      ROS_VERIFY(task_recorder2_utilities::extractDataSamples(recorder_io_.messages_,
                                                              all_variable_names, response.filtered_and_cropped_messages));
    }
    else
    {
      // ...else extract data samples according to request
      ROS_VERIFY(task_recorder2_utilities::extractDataSamples(recorder_io_.messages_,
                                                              request.message_names, response.filtered_and_cropped_messages));
    }
  }

  // always write re-sampled data to file
  // execute in sequence since we need to empty the message buffer afterwards.
  if(!recorder_io_.writeRecordedDataSamples())
  {
    response.info = "Problem writing re-sampled data samples.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  if (recorder_io_.write_out_clmc_data_)
  {
    if (!recorder_io_.writeRecordedDataToCLMCFile())
    {
      response.info = "Problems writing to CLMC file.";
      ROS_ERROR_STREAM(response.info);
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
    }
  }

  // empty buffer after writing data to disc
  recorder_io_.messages_.clear();

  // publish notification
  task_recorder2_msgs::Notification notification;
  notification.description = response.description;
  notification.start = request.crop_start_time;
  notification.end = request.crop_end_time;
  stop_recording_publisher_.publish(notification);

  return true;
}

bool TaskRecorderManager::interruptRecording(task_recorder2_srvs::InterruptRecording::Request& request,
                                             task_recorder2_srvs::InterruptRecording::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    interrupt_recording_requests_[i] = request;
  }

#pragma omp parallel for
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    ROS_VERIFY(task_recorders_[i]->interruptRecording(interrupt_recording_requests_[i], interrupt_recording_responses_[i]));
  }

  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  response.was_recording = true;
  response.last_recorded_time_stamp = ros::TIME_MAX;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    response.info.append(interrupt_recording_responses_[i].info);
    if (interrupt_recording_responses_[i].return_code != interrupt_recording_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = interrupt_recording_responses_[i].return_code;
    }
    if (!interrupt_recording_responses_[i].was_recording)
    {
      response.was_recording = false;
    }
    if (interrupt_recording_responses_[i].last_recorded_time_stamp < response.last_recorded_time_stamp)
    {
      response.last_recorded_time_stamp = interrupt_recording_responses_[i].last_recorded_time_stamp;
    }
  }
  if (response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    response.info.append("Problem when interrupting recording.");
    ROS_DEBUG_STREAM(response.info);
    blackboard_client_.setLoggingUnknown();
    return true;
  }

  interrupt(response.last_recorded_time_stamp, response.was_recording, request.recording);

  blackboard_client_.setLogging(request.recording);
  std::string info = "Continued";
  if (!request.recording)
  {
    info = "Interrupted";
  }
  response.info.append(info + " recording.");
  ROS_DEBUG_STREAM(response.info);

  return true;
}

bool TaskRecorderManager::getInfo(task_recorder2_srvs::GetInfo::Request& request,
                                  task_recorder2_srvs::GetInfo::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  if (!request.description.description.empty())
  {
    ROS_DEBUG("Getting absolute file name of the task recorder manager...");
    if (!recorder_io_.getAbsFileName(request.description, response.file_name))
    {
      response.info = "Could not get file name of requested description. This should never happen.";
      ROS_ERROR_STREAM(response.info);
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
    }
    ROS_DEBUG("Absolute file name is >%s<.", response.file_name.c_str());
  }

  response.info.clear();
  response.sampling_rate = sampling_rate_;
  response.first_recorded_time_stamp = ros::TIME_MIN;
  response.last_recorded_time_stamp = ros::TIME_MAX;
  unsigned int num_idle_recorders = 0;
  response.is_recording = true;
  response.is_streaming = true;
  response.min_num_recorded_messages = std::numeric_limits<unsigned int>::max();
  bool is_recording = response.is_recording;
  bool is_streaming = response.is_streaming;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    ros::Time first;
    ros::Time last;
    unsigned int num_recorded_messages = 0;
    task_recorders_[i]->getTimeStamps(first, last, is_recording, is_streaming, num_recorded_messages);

    if (!is_recording)
    {
      response.is_recording = false;
      num_idle_recorders++;
    }
    if (!is_streaming)
    {
      response.is_streaming = false;
    }

    if (num_recorded_messages < response.min_num_recorded_messages)
    {
      response.min_num_recorded_messages = num_recorded_messages;
    }
    if(first > response.first_recorded_time_stamp)
    {
      response.first_recorded_time_stamp = first;
    }

    if(last < response.last_recorded_time_stamp)
    {
      response.last_recorded_time_stamp = last;
    }
  }

  blackboard_client_.setLogging(response.is_recording);

  // ros::Time last_interrupt;
  // if (getLastInterrupt(last_interrupt))
  // {
  // if (response.last_recorded_time_stamp > last_interrupt)
  // {
  //  ROS_WARN("Corrected last time stamp from >%f< to >%f<.", response.last_recorded_time_stamp.toSec(), last_interrupt.toSec());
  // response.last_recorded_time_stamp = last_interrupt;
  // }
  // }
  // ROS_INFO("first: >%f< last: >%f<.", response.first_recorded_time_stamp.toSec(), response.last_recorded_time_stamp.toSec());

  // error checking
  if (num_idle_recorders != 0 && num_idle_recorders != task_recorders_.size())
  {
    response.info = "Number of idle recorders >" + boost::lexical_cast<std::string>(num_idle_recorders)
                             + "< is invalid. There are >" + boost::lexical_cast<std::string>(task_recorders_.size())
                             + "< task recorders. This should never happen.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  response.info = "First recorded time stamp is at >"
      + boost::lexical_cast<std::string>(response.first_recorded_time_stamp.toSec()) + "<, last time stamp is at >"
      + boost::lexical_cast<std::string>(response.last_recorded_time_stamp.toSec()) + "<. Sampling rate is >"
      + boost::lexical_cast<std::string>(response.sampling_rate) + "< Hz.";
  if (!response.is_recording)
  {
    response.info.append(" Task recorder is currently not recording.");
  }
  if (response.min_num_recorded_messages == 0)
  {
    response.info.append(" There is at least one recorder that has not recorded any messages.");
  }
  // ROS_DEBUG_STREAM(response.info);
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  return true;
}

bool TaskRecorderManager::getDataSample(task_recorder2_srvs::GetDataSample::Request& request,
                                        task_recorder2_srvs::GetDataSample::Response& response)
{
  response.info = "Problem getting data sample for description >"
      + task_recorder2_utilities::getDescription(request.description) + "<. Not implemented yet.";
  ROS_ERROR_STREAM(response.info);
  response.return_code = response.SERVICE_CALL_FAILED;
  return true;

//  // start recording
//  ROS_DEBUG("Getting data sample for description >%s<.", task_recorder2_utilities::getDescription(request.description).c_str());
//  task_recorder2_srvs::StartRecording::Request start_recording_request;
//  start_recording_request.description = request.description;
//  task_recorder2_srvs::StartRecording::Response start_recording_response;
//  ROS_VERIFY(startRecording(start_recording_request, start_recording_response));
//  if (start_recording_response.return_code != start_recording_response.SERVICE_CALL_SUCCESSFUL)
//  {
//    response.info = "Problem starting to record. Cannot get data sample for description >"
//        + task_recorder2_utilities::getDescription(request.description) + "<.";
//    ROS_ERROR_STREAM(response.info);
//    response.return_code = response.SERVICE_CALL_FAILED;
//    return true;
//  }
//
//  // get data sample
//  {
//    boost::mutex::scoped_lock lock(last_combined_data_sample_mutex_);
//    if (!setLastDataSample(start_recording_response.start_time))
//    {
//      response.info = "Could not get last data sample. This should never happen.";
//      ROS_ERROR_STREAM(response.info);
//      response.return_code = response.SERVICE_CALL_FAILED;
//      return true;
//    }
//    response.data_sample = last_combined_data_sample_;
//  }
//
//  // stop streaming
//  task_recorder2_srvs::StopRecording::Request stop_recording_request;
//  stop_recording_request.num_samples = 1;
//  stop_recording_request.crop_start_time = start_recording_response.start_time;
//  stop_recording_request.stop_recording = false;
//  task_recorder2_srvs::StopRecording::Response stop_recording_response;
//  ROS_VERIFY(stopRecording(stop_recording_request, stop_recording_response));
//  if (stop_recording_response.return_code != stop_recording_response.SERVICE_CALL_SUCCESSFUL)
//  {
//    response.info = "Problem stopping to record. Cannot get data sample for description >"
//        + task_recorder2_utilities::getDescription(request.description) + "<.";
//    ROS_ERROR_STREAM(response.info);
//    response.return_code = response.SERVICE_CALL_FAILED;
//    return true;
//  }
//
//  // setup response
//  response.info = std::string("Obtained data sample of topic >" + recorder_io_.topic_name_ + "<. ");
//  ROS_DEBUG_STREAM(response.info);
//  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
//  return true;
}

bool TaskRecorderManager::addDataSamples(task_recorder2_srvs::AddDataSamples::Request& request,
                                         task_recorder2_srvs::AddDataSamples::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  // error checking
  for (unsigned int i = 0; i < request.data_samples.size(); ++i)
  {
    ROS_ASSERT_MSG(request.data_samples[i].data.size() == request.data_samples[i].names.size(),
                   "Provided data samples are inconsistent, sample >%i< contains >%i< values and >%i< names.",
                   (int)i, (int)request.data_samples[i].data.size(), (int)request.data_samples[i].names.size());
  }
  ROS_ASSERT_MSG(!request.data_samples.empty(), "No data samples provided to add.");

  recorder_io_.setDescription(request.description);
  if (!recorder_io_.createDirectories())
  {
    response.info = "Problem creating directories for >" + recorder_io_.topic_name_ + "<. Cannot stop recording.\n";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  recorder_io_.messages_ = request.data_samples;
  if (!recorder_io_.writeRecordedDataSamples())
  {
    response.info = "Could not write recorded data samples when adding data samples.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  response.info = "Added data samples with description >"
      + task_recorder2_utilities::getFileName(request.description) + "<.";

  // publish notification
  task_recorder2_msgs::Notification notification;
  notification.description = request.description;
  notification.start = request.data_samples.front().header.stamp;
  notification.end = request.data_samples.back().header.stamp;
  stop_recording_publisher_.publish(notification);
  return true;
}

bool TaskRecorderManager::readDataSamples(task_recorder2_srvs::ReadDataSamples::Request& request,
                                          task_recorder2_srvs::ReadDataSamples::Response& response)
{
  boost::mutex::scoped_lock lock(service_mutex_);
  if (!recorder_io_.readDataSamples(request.description, response.data_samples))
  {
    response.return_code = response.SERVICE_CALL_FAILED;
    response.info = "Could not read data samples from " + task_recorder2_utilities::getFileName(request.description) + ".";
    return true;
  }
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  response.info = "Read " + boost::lexical_cast<std::string>((int)response.data_samples.size())
                           + " data samples with description >"
                           + task_recorder2_utilities::getFileName(request.description) + "<.";
  return true;
}

bool TaskRecorderManager::setLastDataSample(const ros::Time& time_stamp)
{
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    if (!task_recorders_[i]->getSampleData(time_stamp, data_samples_[i]))
    {
      return false;
    }
  }
  counter_++;

  // boost::mutex::scoped_lock lock(last_combined_data_sample_mutex_);
  if (last_combined_data_sample_.names.empty())
  {
    // allocate memory once
    for (unsigned int i = 0; i < task_recorders_.size(); ++i)
    {
      last_combined_data_sample_.names.insert(last_combined_data_sample_.names.end(),
                                              data_samples_[i].names.begin(),
                                              data_samples_[i].names.end());
    }
    last_combined_data_sample_.data.resize(last_combined_data_sample_.names.size(), 0.0);
  }

  last_combined_data_sample_.header.seq = counter_;
  last_combined_data_sample_.header.stamp = time_stamp;
  unsigned int index = 0;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    for (unsigned int j = 0; j < data_samples_[i].data.size(); ++j)
    {
      last_combined_data_sample_.data[index++] = data_samples_[i].data[j];
    }
  }
  data_sample_publisher_.publish(last_combined_data_sample_);
  return true;
}

void TaskRecorderManager::timerCB(const ros::TimerEvent& timer_event)
{
  setLastDataSample(timer_event.current_expected);
}

void TaskRecorderManager::updateInfo()
{
  task_recorder2_srvs::GetInfo::Request request;
  task_recorder2_srvs::GetInfo::Response response;
  ROS_VERIFY(getInfo(request, response));
  if (response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Problem when updating info : %s", response.info.c_str());
    return;
  }
}

}
