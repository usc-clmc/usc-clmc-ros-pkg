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
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_utilities/data_sample_utilities.h>

// local includes
#include <task_recorder2/task_recorder_manager.h>

namespace task_recorder2
{

TaskRecorderManager::TaskRecorderManager(ros::NodeHandle node_handle) :
    initialized_(false), recorder_io_(node_handle), counter_(-1)
{
  ROS_DEBUG("Creating task recorder manager in namespace >%s<.", node_handle.getNamespace().c_str());
  ROS_VERIFY(recorder_io_.initialize(recorder_io_.node_handle_.getNamespace() + std::string("/data_samples")));
}

bool TaskRecorderManager::initialize()
{
  ROS_VERIFY(read(task_recorders_));
  ROS_ASSERT_MSG(task_recorders_.size() > 0, "No task recorders created. Cannot initialize TaskRecorderManager.");

  data_samples_.resize(task_recorders_.size());
  start_streaming_requests_.resize(task_recorders_.size());
  start_streaming_responses_.resize(task_recorders_.size());
  start_recording_requests_.resize(task_recorders_.size());
  start_recording_responses_.resize(task_recorders_.size());
  stop_recording_requests_.resize(task_recorders_.size());
  stop_recording_responses_.resize(task_recorders_.size());
  interrupt_recording_requests_.resize(task_recorders_.size());
  interrupt_recording_responses_.resize(task_recorders_.size());

  double sampling_rate;
  ROS_VERIFY(usc_utilities::read(recorder_io_.node_handle_, "sampling_rate", sampling_rate));
  ROS_ASSERT(sampling_rate > 0);
  update_timer_period_ = static_cast<double>(1.0) / sampling_rate;
  timer_ = recorder_io_.node_handle_.createTimer(ros::Duration(update_timer_period_), &TaskRecorderManager::timerCB, this);

  const int PUBLISHER_BUFFER_SIZE = 1000;
  data_sample_publisher_ = recorder_io_.node_handle_.advertise<task_recorder2_msgs::DataSample>("data_samples", PUBLISHER_BUFFER_SIZE);
  stop_recording_publisher_ = recorder_io_.node_handle_.advertise<task_recorder2_msgs::Notification>("notification", PUBLISHER_BUFFER_SIZE);

  start_recording_service_server_ = recorder_io_.node_handle_.advertiseService("start_recording", &TaskRecorderManager::startRecording, this);
  stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService("stop_recording", &TaskRecorderManager::stopRecording, this);
  start_streaming_service_server_ = recorder_io_.node_handle_.advertiseService("start_streaming", &TaskRecorderManager::startStreaming, this);
  interrupt_recording_service_server_ = recorder_io_.node_handle_.advertiseService("interrupt_recording", &TaskRecorderManager::interruptRecording, this);
  get_data_sample_service_server_ = recorder_io_.node_handle_.advertiseService("get_data_sample", &TaskRecorderManager::getDataSample, this);

  return (initialized_ = true);
}

bool TaskRecorderManager::startStreaming(task_recorder2::StartStreaming::Request& request,
                                         task_recorder2::StartStreaming::Response& response)
{
  ROS_ASSERT(initialized_);
  for (int i = 0; i < (int)task_recorders_.size(); ++i)
  {
    start_streaming_requests_[i] = request;
    start_streaming_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->startStreaming(start_streaming_requests_[i], start_streaming_responses_[i]));
  }
  response.return_code = task_recorder2::StartStreaming::Response::SERVICE_CALL_SUCCESSFUL;
  return true;
}

bool TaskRecorderManager::startRecording(task_recorder2::StartRecording::Request& request,
                                         task_recorder2::StartRecording::Response& response)
{
  ROS_ASSERT(initialized_);
  recorder_io_.setDescription(request.description);

  response.start_time = ros::Time::now();
  for (int i = 0; i < (int)task_recorders_.size(); ++i)
  {
    start_recording_requests_[i] = request;
    start_recording_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->startRecording(start_recording_requests_[i], start_recording_responses_[i]));

    if(response.start_time < start_recording_responses_[i].start_time)
    {
      response.start_time = start_recording_responses_[i].start_time;
    }
  }

  response.return_code = task_recorder2::StartRecording::Response::SERVICE_CALL_SUCCESSFUL;
  return true;
}

bool TaskRecorderManager::stopRecording(task_recorder2::StopRecording::Request& request,
                                        task_recorder2::StopRecording::Response& response)
{
  ROS_ASSERT(initialized_);
  for (int i = 0; i < (int)task_recorders_.size(); ++i)
  {
    stop_recording_requests_[i] = request;
    stop_recording_requests_[i].message_names.clear();
    stop_recording_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->stopRecording(stop_recording_requests_[i], stop_recording_responses_[i]));

    ROS_ASSERT(stop_recording_responses_[i].return_code == task_recorder2::StopRecording::Response::SERVICE_CALL_SUCCESSFUL);
    response.info.append(stop_recording_responses_[i].info);
    ROS_DEBUG("Got >%i< messages.", (int)stop_recording_responses_[i].filtered_and_cropped_messages.size());
  }

  int num_messages = 0;
  std::vector<std::string> all_variable_names;
  for (int i = 0; i < (int)task_recorders_.size(); ++i)
  {
    // error checking
    ROS_ASSERT(!stop_recording_responses_[i].filtered_and_cropped_messages.empty());
    num_messages = (int)stop_recording_responses_[i].filtered_and_cropped_messages.size();
    if (i < static_cast<int> (task_recorders_.size() - 1))
    {
      ROS_ASSERT(num_messages == (int)stop_recording_responses_[i].filtered_and_cropped_messages.size());
    }
    all_variable_names.insert(all_variable_names.end(),
                              stop_recording_responses_[i].filtered_and_cropped_messages[0].names.begin(),
                              stop_recording_responses_[i].filtered_and_cropped_messages[0].names.end());
  }

  // accumulate all data samples
  recorder_io_.messages_.clear();

  for (int j = 0; j < num_messages; ++j)
  {
    task_recorder2_msgs::DataSample data_sample;
    data_sample.header.seq = j;
    data_sample.header.stamp = stop_recording_responses_[0].filtered_and_cropped_messages[j].header.stamp;
    data_sample.names = all_variable_names;
    for (int i = 0; i < (int)task_recorders_.size(); ++i)
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
    // ...else extract data samples according to request
    ROS_VERIFY(task_recorder2_utilities::extractDataSamples(recorder_io_.messages_, all_variable_names, response.filtered_and_cropped_messages));
    // response.filtered_and_cropped_messages = recorder_io_.messages_;
  }
  else
  {
    // ...else extract data samples according to request
    ROS_VERIFY(task_recorder2_utilities::extractDataSamples(recorder_io_.messages_, request.message_names, response.filtered_and_cropped_messages));
  }

  response.description = stop_recording_responses_[0].description;
  response.return_code = task_recorder2::StopRecording::Response::SERVICE_CALL_SUCCESSFUL;

  // write resampled data to file
  if(recorder_io_.write_out_resampled_data_)
  {
    // boost::thread(boost::bind(&task_recorder2_utilities::TaskRecorderIO<task_recorder2_msgs::DataSample>::writeRecordedDataSamples, recorder_io_));
    ROS_VERIFY(recorder_io_.writeRecordedDataSamples());
  }
  if(recorder_io_.write_out_clmc_data_)
  {
    ROS_VERIFY(recorder_io_.writeRecordedDataToCLMCFile());
  }

  // publish notification
  task_recorder2_msgs::Notification notification;
  notification.description = response.description;
  notification.start = request.crop_start_time;
  notification.end = request.crop_end_time;
  stop_recording_publisher_.publish(notification);

  return true;
}

bool TaskRecorderManager::interruptRecording(task_recorder2::InterruptRecording::Request& request,
                                             task_recorder2::InterruptRecording::Response& response)
{
  ROS_ASSERT(initialized_);
  for (int i = 0; i < (int)task_recorders_.size(); ++i)
  {
    interrupt_recording_requests_[i] = request;
    interrupt_recording_responses_[i] = response;
    ROS_VERIFY(task_recorders_[i]->interruptRecording(interrupt_recording_requests_[i], interrupt_recording_responses_[i]));
    ROS_ASSERT(interrupt_recording_responses_[i].return_code == task_recorder2::InterruptRecording::Response::SERVICE_CALL_SUCCESSFUL);
    response.info.append(interrupt_recording_responses_[i].info);
  }
  ROS_INFO("Interrupted recording topic >%s<.", recorder_io_.topic_name_.c_str());
  response.info = std::string("Stopped recording >" + recorder_io_.topic_name_ + "<. ");
  response.return_code = task_recorder2::InterruptRecording::Response::SERVICE_CALL_SUCCESSFUL;
  return true;
}

bool TaskRecorderManager::getDataSample(task_recorder2::GetDataSample::Request& request,
                                        task_recorder2::GetDataSample::Response& response)
{
  ROS_ASSERT(initialized_);

  // start recording
  ROS_INFO("Getting data sample for description >%s_%i<.", request.description.description.c_str(), request.description.id);
  task_recorder2::StartRecording::Request start_recording_request;
  start_recording_request.description = request.description;
  task_recorder2::StartRecording::Response start_recording_response;
  ROS_VERIFY(startRecording(start_recording_request, start_recording_response));

  // get data sample
  last_combined_data_sample_mutex_.lock();
  ROS_VERIFY_MSG(setLastDataSample(start_recording_response.start_time), "Could not get last data sample. This should never happen.");
  response.data_sample = last_combined_data_sample_;
  last_combined_data_sample_mutex_.unlock();

  // stop streaming
  task_recorder2::StopRecording::Request stop_recording_request;
  stop_recording_request.num_samples = 1;
  stop_recording_request.crop_start_time = start_recording_response.start_time;
  task_recorder2::StopRecording::Response stop_recording_response;
  ROS_VERIFY(stopRecording(stop_recording_request, stop_recording_response));

  // setup response
  ROS_DEBUG("Obtained data sample of topic >%s<.", recorder_io_.topic_name_.c_str());
  response.info = std::string("Obtained data sample of topic >" + recorder_io_.topic_name_ + "<. ");
  response.return_code = task_recorder2::GetDataSample::Response::SERVICE_CALL_SUCCESSFUL;
  return true;
}

bool TaskRecorderManager::setLastDataSample(const ros::Time& time_stamp)
{
  for (int i = 0; i < (int)task_recorders_.size(); ++i)
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
  for (int i = 0; i < (int)task_recorders_.size(); ++i)
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
