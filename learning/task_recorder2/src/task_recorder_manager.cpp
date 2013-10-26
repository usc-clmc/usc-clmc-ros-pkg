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
// #include <boost/thread.hpp>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_msgs/Notification.h>
#include <task_recorder2_utilities/data_sample_utilities.h>

// local includes
#include <task_recorder2/task_recorder_manager.h>

namespace task_recorder2
{

TaskRecorderManager::TaskRecorderManager(ros::NodeHandle node_handle) :
    initialized_(false), recorder_io_(node_handle), sampling_rate_(-1.0), counter_(-1)
{
  ROS_DEBUG("Creating task recorder manager in namespace >%s<.", node_handle.getNamespace().c_str());
  ROS_VERIFY(recorder_io_.initialize(recorder_io_.node_handle_.getNamespace() + std::string("/data_samples")));
}

bool TaskRecorderManager::initialize()
{
  ROS_VERIFY(read(task_recorders_));
  ROS_INFO("Initialized TaskRecorderManager with >%i< recorders.", (int)task_recorders_.size());
  if (task_recorders_.empty())
  {
    ROS_ERROR("No task recorders created. Cannot initialize TaskRecorderManager.");
    return false;
  }

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
  ROS_ASSERT(sampling_rate_ > 0);
  double update_timer_period = static_cast<double>(1.0) / sampling_rate_;
  timer_ = recorder_io_.node_handle_.createTimer(ros::Duration(update_timer_period), &TaskRecorderManager::timerCB, this);

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

  return (initialized_ = true);
}

int TaskRecorderManager::getNumberOfTaskRecorders()
{
  if(!initialized_)
  {
    ROS_WARN("TaskRecorderManager is not initialized. Returning nothing.");
    return 0;
  }
  return static_cast<int>(task_recorders_.size());
}

bool TaskRecorderManager::startStreaming(task_recorder2_srvs::StartStreaming::Request& request,
                                         task_recorder2_srvs::StartStreaming::Response& response)
{
  ROS_ASSERT(initialized_);
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    start_streaming_requests_[i] = request;
    start_streaming_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->startStreaming(start_streaming_requests_[i], start_streaming_responses_[i]));
    // response.info.append(start_streaming_responses_[i].info);
    if (start_streaming_responses_[i].return_code != start_streaming_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = start_streaming_responses_[i].return_code;
    }
  }
  return true;
}

bool TaskRecorderManager::stopStreaming(task_recorder2_srvs::StopStreaming::Request& request,
                                        task_recorder2_srvs::StopStreaming::Response& response)
{
  ROS_ASSERT(initialized_);
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    stop_streaming_requests_[i] = request;
    stop_streaming_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->stopStreaming(stop_streaming_requests_[i], stop_streaming_responses_[i]));
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
  ROS_ASSERT(initialized_);
  recorder_io_.setDescription(request.description);

  response.start_time = ros::TIME_MAX;
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    start_recording_requests_[i] = request;
    start_recording_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->startRecording(start_recording_requests_[i], start_recording_responses_[i]));
    response.info.append(start_recording_responses_[i].info);
    if(start_recording_responses_[i].return_code != start_recording_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = start_recording_responses_[i].return_code;
      ROS_ERROR_STREAM_COND(!start_recording_responses_[i].info.empty(), start_recording_responses_[i].info);
    }
    if((i == 0) || (response.start_time < start_recording_responses_[i].start_time))
    {
      response.start_time = start_recording_responses_[i].start_time;
    }
  }
  if (response.return_code == response.SERVICE_CALL_SUCCESSFUL)
    response.info.append("Started to record at >" + boost::lexical_cast<std::string>(response.start_time.toSec()) + "<.");
  return true;
}

bool TaskRecorderManager::stopRecording(task_recorder2_srvs::StopRecording::Request& request,
                                        task_recorder2_srvs::StopRecording::Response& response)
{
  ROS_ASSERT(initialized_);
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    stop_recording_requests_[i] = request;
    stop_recording_requests_[i].message_names.clear();
    stop_recording_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->stopRecording(stop_recording_requests_[i], stop_recording_responses_[i]));
    response.info.append(stop_recording_responses_[i].info);
    if (stop_recording_responses_[i].return_code != stop_recording_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = stop_recording_responses_[i].return_code;
      ROS_ERROR_STREAM_COND(!stop_streaming_responses_[i].info.empty(), stop_streaming_responses_[i].info);
    }
  }
  if (response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    return true;
  }

  // update description and create directories if needed
  task_recorder2_msgs::Description description = request.description;
  if (description.description.empty())
  {
    description = recorder_io_.getDescription();
  }
  recorder_io_.setDescription(description);
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
  recorder_io_.messages_.clear();
  for (unsigned int j = 0; j < num_messages; ++j)
  {
    task_recorder2_msgs::DataSample data_sample;
    data_sample.header.seq = j;
    data_sample.header.stamp = stop_recording_responses_[0].filtered_and_cropped_messages[j].header.stamp;
    data_sample.names = all_variable_names;
    for (unsigned int i = 0; i < task_recorders_.size(); ++i)
    {
      data_sample.data.insert(data_sample.data.end(),
                              stop_recording_responses_[i].filtered_and_cropped_messages[j].data.begin(),
                              stop_recording_responses_[i].filtered_and_cropped_messages[j].data.end());
    }
    recorder_io_.messages_.push_back(data_sample);
  }

  // if requested names is empty... return all...
  if(request.message_names.empty())
  {
    // extract all data samples
    ROS_VERIFY(task_recorder2_utilities::extractDataSamples(recorder_io_.messages_, all_variable_names, response.filtered_and_cropped_messages));
  }
  else
  {
    // ...else extract data samples according to request
    ROS_VERIFY(task_recorder2_utilities::extractDataSamples(recorder_io_.messages_, request.message_names, response.filtered_and_cropped_messages));
  }

  response.description = recorder_io_.getDescription();

  // always write re-sampled data to file
  if (!request.stop_recording)
  {
    // execute in sequence since we need to empty the message buffer afterwards.
    if(!recorder_io_.writeRecordedDataSamples())
    {
      response.info = "Problem writing re-sampled data samples.";
      ROS_ERROR_STREAM(response.info);
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
    }
    recorder_io_.messages_.clear();
  }
  else
  {
    // execute in separate thread since we do not need to empty the message buffer afterwards.
    boost::thread(boost::bind(&task_recorder2_io::TaskRecorderIO<task_recorder2_msgs::DataSample>::writeRecordedDataSamples, recorder_io_));
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
  ROS_ASSERT(initialized_);
  response.info.clear();
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    interrupt_recording_requests_[i] = request;
    interrupt_recording_responses_[i] = response;
    // this always succeeds
    ROS_VERIFY(task_recorders_[i]->interruptRecording(interrupt_recording_requests_[i], interrupt_recording_responses_[i]));
    response.info.append(interrupt_recording_responses_[i].info);
    if(interrupt_recording_responses_[i].return_code != interrupt_recording_responses_[i].SERVICE_CALL_SUCCESSFUL)
    {
      response.return_code = interrupt_recording_responses_[i].return_code;
    }
  }
  if (response.return_code != response.SERVICE_CALL_SUCCESSFUL)
  {
    response.info.append("Problem when interrupting recording.");
    ROS_DEBUG_STREAM(response.info);
  }
  else
  {
    std::string info = "Continued";
    if (!request.recording)
    {
      info = "Interrupted";
    }
    response.info.append(info + " recording.");
    ROS_DEBUG_STREAM(response.info);
  }
  return true;
}

bool TaskRecorderManager::getInfo(task_recorder2_srvs::GetInfo::Request& request,
                                  task_recorder2_srvs::GetInfo::Response& response)
{
  ROS_ASSERT(initialized_);
  if(!request.description.description.empty())
  {
    ROS_INFO("Getting info...");
    if(!recorder_io_.getAbsFileName(request.description, response.file_name))
    {
      response.info = "Could not get file name of requested description. This should never happen.";
      ROS_ERROR_STREAM(response.info);
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
    }
    ROS_INFO("Getting info... done >%s<", response.file_name.c_str());
  }

  response.sampling_rate = sampling_rate_;
  response.first_recorded_time_stamp = ros::TIME_MIN;
  response.last_recorded_time_stamp = ros::TIME_MAX;
  unsigned int num_active_recorders = 0;
  response.is_recording = true;
  response.info.clear();
  bool is_recording = response.is_recording;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    ros::Time first;
    ros::Time last;
    is_recording = task_recorders_[i]->getTimeStamps(first, last);
    if (!is_recording)
    {
      response.is_recording = false;
      // ROS_ERROR_STREAM("Recorder >" + boost::lexical_cast<std::string>(i) + "< is not recording.");
    }
    else
    {
      num_active_recorders++;
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

  // error checking
  if(num_active_recorders != 0 && num_active_recorders != task_recorders_.size())
  {
    response.info = "Number of active recorders >" + boost::lexical_cast<std::string>(num_active_recorders)
                         + "< is invalid. There are >" + boost::lexical_cast<std::string>((int)task_recorders_.size())
                         + "< task recorders. This should never happen.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  response.info = "Task recorder is currently recording. First time stamp is at >"
      + boost::lexical_cast<std::string>(response.first_recorded_time_stamp.toSec()) + "<, last time stamp is at >"
      + boost::lexical_cast<std::string>(response.last_recorded_time_stamp.toSec()) + "<. Sampling rate is >"
      + boost::lexical_cast<std::string>(response.sampling_rate) + "< Hz.";
  if (!response.is_recording)
  {
    response.info = "Task recorder is currently not recording.";
  }
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  return true;
}

bool TaskRecorderManager::getDataSample(task_recorder2_srvs::GetDataSample::Request& request,
                                        task_recorder2_srvs::GetDataSample::Response& response)
{
  ROS_ASSERT(initialized_);

  // start recording
  ROS_DEBUG("Getting data sample for description >%s<.", task_recorder2_utilities::getDescription(request.description).c_str());
  task_recorder2_srvs::StartRecording::Request start_recording_request;
  start_recording_request.description = request.description;
  task_recorder2_srvs::StartRecording::Response start_recording_response;
  ROS_VERIFY(startRecording(start_recording_request, start_recording_response));
  if (start_recording_response.return_code != start_recording_response.SERVICE_CALL_SUCCESSFUL)
  {
    response.info = "Problem starting to record. Cannot get data sample for description >"
        + task_recorder2_utilities::getDescription(request.description) + "<.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  // get data sample
  last_combined_data_sample_mutex_.lock();
  if(!setLastDataSample(start_recording_response.start_time))
  {
    last_combined_data_sample_mutex_.unlock();
    response.info = "Could not get last data sample. This should never happen.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }
  response.data_sample = last_combined_data_sample_;
  last_combined_data_sample_mutex_.unlock();

  // stop streaming
  task_recorder2_srvs::StopRecording::Request stop_recording_request;
  stop_recording_request.num_samples = 1;
  stop_recording_request.crop_start_time = start_recording_response.start_time;
  task_recorder2_srvs::StopRecording::Response stop_recording_response;
  ROS_VERIFY(stopRecording(stop_recording_request, stop_recording_response));
  if (stop_recording_response.return_code != stop_recording_response.SERVICE_CALL_SUCCESSFUL)
  {
    response.info = "Problem stopping to record. Cannot get data sample for description >"
        + task_recorder2_utilities::getDescription(request.description) + "<.";
    ROS_ERROR_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    return true;
  }

  // setup response
  response.info = std::string("Obtained data sample of topic >" + recorder_io_.topic_name_ + "<. ");
  ROS_DEBUG_STREAM(response.info);
  response.return_code = response.SERVICE_CALL_SUCCESSFUL;
  return true;
}

bool TaskRecorderManager::addDataSamples(task_recorder2_srvs::AddDataSamples::Request& request,
                                         task_recorder2_srvs::AddDataSamples::Response& response)
{
  ROS_ASSERT(initialized_);
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
  response.info = "Added data samples with description >" + task_recorder2_utilities::getFileName(request.description) + "<.";

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
  ROS_ASSERT(initialized_);
  if(!recorder_io_.readDataSamples(request.description, response.data_samples))
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

  // TODO: think about how to avoid memory re-allocation
  last_combined_data_sample_.names.clear();
  last_combined_data_sample_.data.clear();
  last_combined_data_sample_.header.seq = counter_;
  last_combined_data_sample_.header.stamp = time_stamp;
  for (unsigned int i = 0; i < task_recorders_.size(); ++i)
  {
    last_combined_data_sample_.names.insert(last_combined_data_sample_.names.end(), data_samples_[i].names.begin(), data_samples_[i].names.end());
    last_combined_data_sample_.data.insert(last_combined_data_sample_.data.end(), data_samples_[i].data.begin(), data_samples_[i].data.end());
  }
  data_sample_publisher_.publish(last_combined_data_sample_);
  return true;
}

void TaskRecorderManager::timerCB(const ros::TimerEvent& timer_event)
{
  last_combined_data_sample_mutex_.lock();
  setLastDataSample(timer_event.current_expected);
  last_combined_data_sample_mutex_.unlock();
}

}
