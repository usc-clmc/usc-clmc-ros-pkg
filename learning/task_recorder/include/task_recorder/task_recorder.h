/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \file    task_recorder.h

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

#ifndef TASK_RECORDER1_TASK_RECORDER_H_
#define TASK_RECORDER1_TASK_RECORDER_H_

// system includes
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

// ros includes
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/bspline.h>

// local includes
#include <task_recorder/task_recorder_io.h>
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/StartRecording.h>
#include <task_recorder/accumulator.h>
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder
{

template<class MessageType, class StopServiceType>
  class TaskRecorder
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
    typedef boost::shared_ptr<StopServiceType const> StopServiceTypeConstPtr;

    /*!
     * @return
     */
    TaskRecorder() :
      initialized_(false), is_recording_(false) {};
    virtual ~TaskRecorder() {};

    /*!
     * @param node_handle
     * @param topic_name
     * @return True on success, otherwise False
     */
    bool initializeBase(ros::NodeHandle& node_handle,
                        const std::string& topic_name);

    /*!
     */
    void startRecording();

    /*!
     */
    void stopRecording();

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool startRecording(task_recorder::StartRecording::Request& request,
                        task_recorder::StartRecording::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool stopRecording(typename StopServiceType::Request& request,
                       typename StopServiceType::Response& response);


  protected:

    /*!
     */
    bool initialized_;
    bool is_recording_;

    /*!
     */
    TaskRecorderIO<MessageType> recorder_io_;

    /*!
     * @param start_time
     * @param end_time
     * @param num_samples
     * @param filter_and_cropped_messages
     * @param message_names
     * @param times
     * @param data
     * @return True on success, otherwise False
     */
    virtual bool filterAndCrop(const ros::Time& start_time,
                               const ros::Time& end_time,
                               int num_samples,
                               std::vector<MessageType>& filter_and_cropped_messages,
                               std::vector<std::string>& message_names,
                               std::vector<ros::Time>& times,
                               std::vector<double>& data);

    /*!
     * @param message
     * @return True on success, otherwise False
     */
    virtual bool transformMessages(MessageType& message);

    /*!
     * @param vector_of_accumulated_trial_statistics
     * @return True on success, otherwise False
     */
    virtual bool getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics);

    /*!
     * @param accumulated_trial_statistics
     */
    virtual void setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics) = 0;

    /*!
     * @param signal_index
     * @param signal_name
     */
    virtual void getSignalNames(const int signal_index,
                                std::string& signal_name) = 0;

    virtual void getVariables(const MessageType& message, std::vector<double>& variables) = 0;
    virtual void setVariables(MessageType& message, const std::vector<double>& variables) = 0;
    virtual int getNumVariables() = 0;

  private:
    /*!
     */
    ros::ServiceServer start_recording_service_server_;
    ros::ServiceServer stop_recording_service_server_;

    /*!
     */
    ros::Subscriber subscriber_;

    /*!
     */
    boost::mutex mutex_;
    bool logging_;

    task_recorder::Accumulator accumulator_;

    /*!
     */
    void recordMessagesCallback(const MessageTypeConstPtr message);

    bool resample(std::vector<MessageType>& messages,
                                        const ros::Time& start_time,
                                        const ros::Time& end_time,
                                        const int num_samples,
                                        std::vector<MessageType>& resampled_messages,
                                        bool use_bspline=false);

  };

template<class MessageType, class StopServiceType>
  bool TaskRecorder<MessageType, StopServiceType>::initializeBase(ros::NodeHandle& node_handle,
                                                                  const std::string& topic_name)
  {
    if(!recorder_io_.initialize(node_handle, topic_name))
    {
      ROS_ERROR("Could not initialize base of the task recorder.");
      return (initialized_ = false);
    }

    std::string name = topic_name;
    if(!getTopicName(name))
    {
      ROS_ERROR("Could not obtain topic name from name >%s<. Could not initialize base of the task recorder.", name.c_str());
      return (initialized_ = false);
    }
    std::string start_recording_service_name = std::string("start_recording_") + name;
    std::string stop_recording_service_name = std::string("stop_recording_") + name;

    start_recording_service_server_ = recorder_io_.node_handle_.advertiseService(start_recording_service_name,
                                                                                 &TaskRecorder<MessageType, StopServiceType>::startRecording, this);
    stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService(stop_recording_service_name,
                                                                                &TaskRecorder<MessageType, StopServiceType>::stopRecording, this);
    return (initialized_ = true);
  }

template<class MessageType, class StopServiceType>
  void TaskRecorder<MessageType, StopServiceType>::recordMessagesCallback(const MessageTypeConstPtr message)
  {
    mutex_.lock();
    if (logging_)
    {
      MessageType msg = *message;
      transformMessages(msg);
      //double delay = (ros::Time::now() - msg.header.stamp).toSec();
      //ROS_INFO("Delay = %f", delay);
      recorder_io_.messages_.push_back(msg);
    }
    mutex_.unlock();
  }

template<class MessageType, class StopServiceType>
  void TaskRecorder<MessageType, StopServiceType>::startRecording()
  {
    ROS_INFO("Start recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, 10000, &TaskRecorder<MessageType, StopServiceType>::recordMessagesCallback, this);
    mutex_.lock();
    logging_ = true;
    recorder_io_.messages_.clear();
    mutex_.unlock();
  }

template<class MessageType, class StopServiceType>
  void TaskRecorder<MessageType, StopServiceType>::stopRecording()
  {
    ROS_INFO("Stop recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    mutex_.lock();
    logging_ = false;
    mutex_.unlock();
    subscriber_.shutdown();
  }

template<class MessageType, class StopServiceType>
  bool TaskRecorder<MessageType, StopServiceType>::startRecording(task_recorder::StartRecording::Request& request,
                                                                  task_recorder::StartRecording::Response& response)
  {
    startRecording();
    recorder_io_.setId(request.id);
    response.return_code = task_recorder::StartRecording::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType, class StopServiceType>
  bool TaskRecorder<MessageType, StopServiceType>::stopRecording(typename StopServiceType::Request& request,
                                                                 typename StopServiceType::Response& response)
  {

    stopRecording();
    std::vector<MessageType> filtered_and_cropped_messages;
    std::vector<ros::Time> times;
    std::vector<double> data;

    if (!filterAndCrop(request.crop_start_time, request.crop_end_time, request.num_samples, filtered_and_cropped_messages, request.message_names, times, data))
    {
      ROS_ERROR("Could not filter and crop messages.");
      response.return_code = StopServiceType::Response::SERVICE_CALL_FAILED;
      return true;
    }

    // write out data files
    boost::thread(boost::bind(&TaskRecorderIO<MessageType>::writeRecordedData, recorder_io_));

    // write out statistics files
    std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> > vector_of_accumulated_trial_statistics;
    ROS_VERIFY(getAccumulatedTrialStatistics(vector_of_accumulated_trial_statistics));
    boost::thread(boost::bind(&TaskRecorderIO<MessageType>::writeStatistics, recorder_io_, vector_of_accumulated_trial_statistics));

    response.filtered_and_cropped_messages = filtered_and_cropped_messages;
    response.times = times;
    response.data = data;
    response.return_code = StopServiceType::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType, class StopServiceType>
bool TaskRecorder<MessageType, StopServiceType>::filterAndCrop(const ros::Time& start_time,
                           const ros::Time& end_time,
                           int num_samples,
                           std::vector<MessageType>& filter_and_cropped_messages,
                           std::vector<std::string>& message_names,
                           std::vector<ros::Time>& times,
                           std::vector<double>& data)
{
  int num_messages = recorder_io_.messages_.size();
  if (num_messages == 0)
  {
    ROS_ERROR("Zero messages have been logged.");
    return false;
  }

  // figure out when our data starts and ends
  ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
  ros::Time our_end_time = recorder_io_.messages_[num_messages - 1].header.stamp;
  int index = 0;
  while (our_end_time.toSec() < 1e-6)
  {
    index++;
    our_end_time = recorder_io_.messages_[num_messages - (1 + index)].header.stamp;
  }

  if (our_start_time > start_time || our_end_time < end_time)
  {
    ROS_ERROR("Requested times have not been recorded!");
    ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
    ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
    return false;
  }

  // fit bspline and resample the position and effort trajectories and compute the velocities
  ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples, filter_and_cropped_messages, true));
  ROS_ASSERT(static_cast<int>(filter_and_cropped_messages.size()) == num_samples);

  recorder_io_.messages_ = filter_and_cropped_messages;
  return true;
}

template<class MessageType, class StopServiceType>
bool TaskRecorder<MessageType, StopServiceType>::resample(std::vector<MessageType>& messages,
                                    const ros::Time& start_time,
                                    const ros::Time& end_time,
                                    const int num_samples,
                                    std::vector<MessageType>& resampled_messages,
                                    bool use_bspline)
{
  int num_vars = getNumVariables();

  ROS_VERIFY(!messages.empty());

  // first crop
  //ROS_VERIFY(crop<MessageType>(messages, start_time, end_time));
  // then remove duplicates
  ROS_VERIFY(removeDuplicates<MessageType>(messages));

  ROS_VERIFY(recorder_io_.writeRawData());

  int num_messages = static_cast<int> (messages.size());

  //ROS_VERIFY(accumulator_.initialize(num_vars, num_samples));

  // compute mean dt of the provided time stamps
  double dts[num_messages - 1];
  double mean_dt = 0.0;

  std::vector<double> input_vector(num_messages);
  input_vector[0] = messages[0].header.stamp.toSec();
  for (int i = 0; i < num_messages - 1; i++)
  {
    dts[i] = messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec();
    mean_dt += dts[i];
    input_vector[i + 1] = input_vector[i] + dts[i];
  }
  mean_dt /= static_cast<double> (num_messages - 1);

  ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

  double wave_length = interval.toSec() * static_cast<double> (2.0);

  resampled_messages.clear();
  std::vector<double> input_querry(num_samples);
  for (int i = 0; i < num_samples; i++)
  {
    MessageType msg = messages[0];
    msg.header.seq = i;
    msg.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
    input_querry[i] = msg.header.stamp.toSec();
    resampled_messages.push_back(msg);
  }

  std::vector<std::vector<double> > variables;
  std::vector<std::vector<double> > variables_resampled;

  variables.resize(num_vars);
  variables_resampled.resize(num_vars);

  std::vector<double> temp_vars;
  temp_vars.resize(num_vars, 0.0);

  for (int j = 0; j < num_messages; ++j)
  {
    getVariables(messages[j], temp_vars);
    for (int i=0; i<num_vars; ++i)
    {
      variables[i].push_back(temp_vars[i]);
    }
  }

  for (int i=0; i<num_vars; ++i)
  {
    if (use_bspline)
    {
      if (!usc_utilities::resample(input_vector, variables[i], wave_length, input_querry, variables_resampled[i], false))
      {
        ROS_ERROR("Could not resample variables, splining failed.");
        return false;
      }
    }
    else
    {
      ROS_VERIFY(usc_utilities::resampleLinear(input_vector, variables[i], input_querry, variables_resampled[i]));
    }
  }

  for (int j = 0; j < num_samples; ++j)
  {
    for (int i=0; i<num_vars; ++i)
    {
      temp_vars[i] = variables_resampled[i][j];
      setVariables(resampled_messages[j], temp_vars);
    }
  }

  for (int i=0; i<num_vars; ++i)
  {
    //ROS_VERIFY(accumulator_.add(i, variables_resampled[i]));
  }
  return true;
}

template<class MessageType, class StopServiceType>
bool TaskRecorder<MessageType, StopServiceType>::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
{
  vector_of_accumulated_trial_statistics.clear();
  std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics;
  ROS_VERIFY(accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
  setMessageNames(accumulated_trial_statistics);
  vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
  return true;
}

template<class MessageType, class StopServiceType>
bool TaskRecorder<MessageType, StopServiceType>::transformMessages(MessageType& message)
{
  return true;
}

}

#endif /* TASK_RECORDER_H_ */
