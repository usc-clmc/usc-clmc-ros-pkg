/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_recorder_manager_client.h

  \author	Peter Pastor
  \date		Jun 8, 2011

 *********************************************************************/

#ifndef TASK_RECORDER_MANAGER_CLIENT_H_
#define TASK_RECORDER_MANAGER_CLIENT_H_

// system includes
#include <string>
#include <ros/ros.h>

#include <usc_utilities/assert.h>
#include <visualization_msgs/Marker.h>

#include <blackboard/blackboard_client.h>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSample.h>

#include <blackboard/blackboard_client.h>

// local includes

namespace task_recorder2_client
{

class TaskRecorderManagerClient
{

public:

  /*! Constructor
   */
  TaskRecorderManagerClient(bool block_until_services_are_available = false);
  /*! Destructor
   */
  virtual ~TaskRecorderManagerClient() {};

  /*!
   * @return True if all services are ready, otherwise False
   */
  bool checkForServices();

  /*!
   * Waits until all services are online
   */
  void waitForServices();

  /*!
   * @return True if all services are online, otherwise False
   */
  bool servicesAreReady() const
  {
    return is_online_;
  }

  /*!
   * @return
   */
  bool startStreaming();

  /*!
   * @return
   */
  bool stopStreaming();

  /*!
   * Start recording data
   * @param description
   * @param id
   * @param start_time will contain the time stamp at which the last recorded started to record
   * @return True on success, otherwise False
   */
  bool startRecording(const std::string description,
                      const int id,
                      ros::Time& start_time);

  /*!
   * Start recording data
   * @param description
   * @param start_time will contain the time stamp at which the last recorded started to record
   * @return True on success, otherwise False
   */
  bool startRecording(const task_recorder2_msgs::Description& description,
                      ros::Time& start_time);

  /*!
   * Stop recording data and resample ALL recorded data (until end_time) traces such that they contain num_samples samples
   * @param start_time contains the start time at which the data will be cropped
   * @param end_time contains the end time at which the data will be cropped
   * @param num_samples
   * @param messages
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const ros::Time& start_time,
                     const ros::Time& end_time,
                     const int num_samples,
                     std::vector<task_recorder2_msgs::DataSample>& messages,
                     const bool stop_recording = true);

  /*!
   * Stop recording data and resample recorded data traces (until end_time) that matches the provided variable_names,
   * such that they contain num_samples samples
   * @param start_time contains the start time at which the data will be cropped
   * @param end_time contains the end time at which the data will be cropped
   * @param num_samples
   * @param message_names
   * @param messages
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const ros::Time& start_time,
                     const ros::Time& end_time,
                     const int num_samples,
                     const std::vector<std::string>& message_names,
                     std::vector<task_recorder2_msgs::DataSample>& messages,
                     const bool stop_recording = true);

  /*!
   * Stop recording data and resample recorded data traces (until end_time) that matches the provided variable_names,
   * at the default sampling rate (see param "sampling_rate" on param server e.g. arm_task_recorder_manager.yaml)
   * @param start_time contains the start time at which the data will be cropped
   * @param end_time contains the end time at which the data will be cropped
   * @param message_names
   * @param messages
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const ros::Time& start_time,
                     const ros::Time& end_time,
                     const std::vector<std::string>& message_names,
                     std::vector<task_recorder2_msgs::DataSample>& messages,
                     const bool stop_recording = true);

  /*!
   * Stop recording data (right now) and resample data traces that matches the provided variable_names,
   * at the default sampling rate (see param "sampling_rate" on param server e.g. arm_task_recorder_manager.yaml)
   * @param start_time contains the start time at which the data will be cropped
   * @param message_names
   * @param messages
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const ros::Time& start_time,
                     const std::vector<std::string>& message_names,
                     std::vector<task_recorder2_msgs::DataSample>& messages,
                     const bool stop_recording = true);


  /*!
   * Stop recording data (right now) and resample data traces that matches the provided variable_names,
   * at the default sampling rate (see param "sampling_rate" on param server e.g. arm_task_recorder_manager.yaml)
   * @param message_names
   * @param messages
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const std::vector<std::string>& message_names,
                     std::vector<task_recorder2_msgs::DataSample>& messages,
                     const bool stop_recording = true);

  /*!
   * Stop recording data (right now) and resample ALL data traces
   * @param messages
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(std::vector<task_recorder2_msgs::DataSample>& messages,
                     const bool stop_recording = true)
  {
    std::vector<std::string> no_message_names;
    return stopRecording(no_message_names, messages, stop_recording);
  }

  /*!
   * @param start_time contains the start time at which the data will be cropped
   * @param end_time contains the end time at which the data will be cropped
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const ros::Time& start_time,
                     const ros::Time& end_time,
                     const bool stop_recording = true)
  {
    std::vector<task_recorder2_msgs::DataSample> no_messages;
    std::vector<std::string> no_message_names;
    return stopRecording(start_time, end_time, no_message_names, no_messages, stop_recording);
  }

  /*!
   * Stop recording data (right now) and resample ALL data traces
   * @param stop_recording If set, the task recorder actually stops logging messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const bool stop_recording = true)
  {
    std::vector<task_recorder2_msgs::DataSample> no_messages;
    std::vector<std::string> no_message_names;
    return stopRecording(no_message_names, no_messages, stop_recording);
  }

  /*!
   * @param recording
   * @return True on success, otherwise False
   */
  bool interruptRecording(const bool recording);

  /*!
   * @param description
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool getDataSample(const task_recorder2_msgs::Description& description,
                     task_recorder2_msgs::DataSample& data_sample);
  bool getDataSample(const task_recorder2_msgs::Description& description)
  {
    task_recorder2_msgs::DataSample data_sample;
    bool result = getDataSample(description, data_sample);
    return result;
  }

  /*!
   * @param description
   * @param data_samples
   * @return True on success, otherwise False
   */
  bool addDataSamples(const task_recorder2_msgs::Description& description,
                      const std::vector<task_recorder2_msgs::DataSample>& data_samples);
  bool addDataSample(const task_recorder2_msgs::Description& description,
                     const task_recorder2_msgs::DataSample& data_sample)
  {
    std::vector<task_recorder2_msgs::DataSample> data_samples;
    data_samples.push_back(data_sample);
    return addDataSamples(description, data_samples);
  }

  /*!
   * @param description
   * @param data_samples
   * @return True on success, otherwise False
   */
  bool readDataSamples(const task_recorder2_msgs::Description& description,
                       std::vector<task_recorder2_msgs::DataSample>& data_samples);

  /*!
   * @param description
   * @param abs_file_name
   * @return True on success, otherwise False
   */
  bool getInfo(const task_recorder2_msgs::Description& description,
               std::string& abs_file_name);

  /*!
   * @param sampling_rate
   * @return True on success, otherwise False
   */
  bool getInfo(double& sampling_rate)
  {
    bool is_recording = false;
    ros::Time first;
    ros::Time last;
    return getInfo(is_recording, first, last, sampling_rate);
  }

private:

  /*!
   */
  ros::NodeHandle node_handle_;
  bool is_online_;

  /*!
   */
  ros::ServiceClient start_streaming_service_client_;
  ros::ServiceClient stop_streaming_service_client_;
  ros::ServiceClient start_recording_service_client_;
  ros::ServiceClient stop_recording_service_client_;
  ros::ServiceClient interrupt_recording_service_client_;
  ros::ServiceClient get_data_sample_service_client_;
  ros::ServiceClient add_data_samples_service_client_;
  ros::ServiceClient read_data_samples_service_client_;
  ros::ServiceClient get_info_service_client_;

  /*!
   */
  bool getInfo(bool& is_recording,
               ros::Time& first,
               ros::Time& last,
               double& sampling_rate);

  bool stoppable(const bool is_recording,
                 const ros::Time& first,
                 const ros::Time& last);


  blackboard::RightBlackBoardClient blackboard_client_;

};

}

#endif /* TASK_RECORDER_MANAGER_CLIENT_H_ */
