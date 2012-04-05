/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    task_recorder_client.h

 \author  Mrinal Kalakrishnan, Peter Pastor
 \date    Jul 14, 2010

 *********************************************************************/

#ifndef TASK_RECORDER_CLIENT_H_
#define TASK_RECORDER_CLIENT_H_

// system includes
#include <ros/ros.h>
#include <string>

#include <usc_utilities/assert.h>
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>

// local includes
#include <task_recorder2/StartRecording.h>
#include <task_recorder2/StopRecording.h>

namespace task_recorder2
{

template<class MessageType>
  class TaskRecorderClient
  {
  public:

    /*! Constructor
     */
    TaskRecorderClient(ros::NodeHandle node_handle) :
      node_handle_(node_handle) {};
    /*! Destructor
     */
    virtual ~TaskRecorderClient() {};

    /*!
     * @param topic_name
     * @param node_name
     * @return True on success, otherwise False
     */
    bool initialize(const std::string& topic_name,
                    const std::string node_name = "TaskRecorder");

    /*!
     * Start recording data
     * @param id
     * @param description
     * @return True on success, otherwise False
     */
    bool startRecording(const int id = 0, const std::string description = "data");

    /*!
     * Start recording data
     * @param description
     * @return True on success, otherwise False
     */
    bool startRecording(const task_recorder2_msgs::Description& description);

    /*!
     * Stop recording data
     * @param start_time
     * @param end_times
     * @param num_samples
     * @return
     */
    bool stopRecording(const ros::Time& start_time,
                       const ros::Time& end_time,
                       const int num_samples);

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
                       const int num_samples,
                       const std::vector<std::string>& message_names);

    /*!
     */
    std::vector<task_recorder2_msgs::DataSample> messages_;

  private:

    /*!
     */
    ros::NodeHandle node_handle_;

    /*!
     */
    ros::ServiceClient start_recording_service_client_;
    ros::ServiceClient stop_recording_service_client_;

  };

template<class MessageType>
  bool TaskRecorderClient<MessageType>::initialize(const std::string& topic_name,
                                                   const std::string node_name)
  {
    std::string service_name = topic_name;
    ROS_VERIFY(task_recorder2_utilities::getTopicName(service_name));

    std::string start_recording_service_name = "/" + node_name + "/start_recording_" + service_name;
    std::string stop_recording_service_name = "/" + node_name + "/stop_recording_" + service_name;

    start_recording_service_client_ = node_handle_.serviceClient<task_recorder2::StartRecording> (start_recording_service_name);
    stop_recording_service_client_ = node_handle_.serviceClient<task_recorder2::StopRecording> (stop_recording_service_name);

    return true;
  }

template<class MessageType>
  bool TaskRecorderClient<MessageType>::startRecording(const int id, const std::string description)
  {
    task_recorder2_msgs::Description task_description;
    task_description.id = id;
    task_description.description.assign(description);
    return startRecording(task_description);
  }

template<class MessageType>
  bool TaskRecorderClient<MessageType>::startRecording(const task_recorder2_msgs::Description& description)
  {
    task_recorder2::StartRecording::Request start_request;
    start_request.description = description;
    task_recorder2::StartRecording::Response start_response;

    bool service_online = false;
    ROS_DEBUG("Waiting for >%s< ...", start_recording_service_client_.getService().c_str());
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
    ROS_DEBUG("Calling >%s< ...", start_recording_service_client_.getService().c_str());
    return start_recording_service_client_.call(start_request, start_response);
  }

template<class MessageType>
  bool TaskRecorderClient<MessageType>::stopRecording(const ros::Time& start_time,
                                                      const ros::Time& end_time,
                                                      const int num_samples)
  {
    std::vector<std::string> no_message_names;
    return stopRecording(start_time, end_time, num_samples, no_message_names);
  }

template<class MessageType>
  bool TaskRecorderClient<MessageType>::stopRecording(const ros::Time& start_time,
                                                      const ros::Time& end_time,
                                                      const int num_samples,
                                                      const std::vector<std::string>& message_names)
  {
    messages_.clear();

    task_recorder2::StopRecording::Request stop_request;
    task_recorder2::StopRecording::Response stop_response;
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
    if (stop_response.return_code != task_recorder2::StopRecording::Response::SERVICE_CALL_SUCCESSFUL)
    {
      ROS_ERROR("Stop recording service >%s< failed with return code >%i<.", stop_recording_service_client_.getService().c_str(), stop_response.return_code);
      return false;
    }

    messages_ = stop_response.filtered_and_cropped_messages;
    return true;
  }

}

#endif /* TASK_RECORDER_CLIENT_H_ */
