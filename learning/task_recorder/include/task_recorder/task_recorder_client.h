/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef TASK_RECORDER1_TASK_RECORDER_CLIENT_H_
#define TASK_RECORDER1_TASK_RECORDER_CLIENT_H_

// system includes
#include <ros/ros.h>
#include <string>

#include <usc_utilities/assert.h>

// local includes
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/StartRecording.h>

namespace task_recorder
{

template<class MessageType, class StopServiceType>
  class TaskRecorderClient
  {
  public:

    /*! Constructor
     */
    TaskRecorderClient() {};
    /*! Destructor
     */
    virtual ~TaskRecorderClient() {};

    /*!
     * @param node_handle
     * @param topic_name
     * @return True on success, otherwise False
     */
    bool initialize(ros::NodeHandle& node_handle,
                    const std::string& topic_name);

    /**
     * Start recording data
     * @return True on success, otherwise False
     */
    bool startRecording();

    /**
     * Stop recording data
     * @param start_time
     * @param end_times
     * @param num_samples
     * @return
     */
    bool stopRecording(const ros::Time& start_time,
                       const ros::Time& end_time,
                       int num_samples);
    /**
     * Stop recording data
     * @param start_time
     * @param end_times
     * @param num_samples
     * @param message_names
     * @return
     */
    bool stopRecording(const ros::Time& start_time,
                       const ros::Time& end_time,
                       int num_samples,
                       const std::vector<std::string>& message_names);

    /*!
     */
    std::vector<MessageType> messages_;
    std::vector<ros::Time> times_;
    std::vector<double> data_;

  private:

    ros::NodeHandle node_handle_;
    std::string topic_name_;

    ros::ServiceClient start_recording_service_client_;
    ros::ServiceClient stop_recording_service_client_;

  };

template<class MessageType, class StopServiceType>
  bool TaskRecorderClient<MessageType, StopServiceType>::initialize(ros::NodeHandle& node_handle,
                                                                    const std::string& topic_name)
  {
    node_handle_ = node_handle;

    topic_name_ = topic_name;
    std::string service_name = topic_name;
    ROS_VERIFY(task_recorder::getTopicName(service_name));

    std::string start_recording_service_name = "/TaskRecorder/start_recording_" + service_name;
    std::string stop_recording_service_name = "/TaskRecorder/stop_recording_" + service_name;

    start_recording_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    stop_recording_service_client_ = node_handle_.serviceClient<StopServiceType> (stop_recording_service_name);

    return true;
  }

template<class MessageType, class StopServiceType>
  bool TaskRecorderClient<MessageType, StopServiceType>::startRecording()
  {
    task_recorder::StartRecording::Request start_request;
    task_recorder::StartRecording::Response start_response;

    start_request.topic_names.clear();
    start_request.topic_names.push_back(topic_name_);

    bool service_online = false;
    while (ros::ok() && !service_online)
    {
      if (!start_recording_service_client_.waitForExistence(ros::Duration(1.0)))
      {
        ROS_WARN("Waiting for >%s< ...", start_recording_service_client_.getService().c_str());
      }
      else
      {
        service_online = true;
      }
    }
    return start_recording_service_client_.call(start_request, start_response);
  }

template<class MessageType, class StopServiceType>
  bool TaskRecorderClient<MessageType, StopServiceType>::stopRecording(const ros::Time& start_time,
                                                                       const ros::Time& end_time,
                                                                       int num_samples)
  {
    std::vector<std::string> no_message_names;
    return stopRecording(start_time, end_time, num_samples, no_message_names);
  }

template<class MessageType, class StopServiceType>
  bool TaskRecorderClient<MessageType, StopServiceType>::stopRecording(const ros::Time& start_time,
                                                                       const ros::Time& end_time,
                                                                       int num_samples,
                                                                       const std::vector<std::string>& message_names)
  {
    messages_.clear();

    typename StopServiceType::Request stop_request;
    typename StopServiceType::Response stop_response;
    stop_request.crop_start_time = start_time;
    stop_request.crop_end_time = end_time;
    stop_request.num_samples = num_samples;
    stop_request.message_names = message_names;

    bool service_online = false;
    while (ros::ok() && !service_online)
    {
      if (!stop_recording_service_client_.waitForExistence(ros::Duration(1.0)))
      {
        ROS_WARN("Waiting for >%s< ...", stop_recording_service_client_.getService().c_str());
      }
      else
      {
        service_online = true;
      }
    }

    ROS_VERIFY(stop_recording_service_client_.call(stop_request, stop_response));
    if(stop_response.return_code != StopServiceType::Response::SERVICE_CALL_SUCCESSFUL)
    {
      ROS_ERROR("Stop recording service >%s< failed with return code >%i<.", stop_recording_service_client_.getService().c_str(), stop_response.return_code);
      return false;
    }

    messages_ = stop_response.filtered_and_cropped_messages;
    times_ = stop_response.times;
    data_ = stop_response.data;
    return true;
  }

}

#endif /* TASK_RECORDER_CLIENT_H_ */
