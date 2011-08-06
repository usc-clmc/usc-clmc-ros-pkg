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

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSample.h>

// local includes

namespace task_recorder2
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
   * Start recording data
   * @param description
   * @param id
   * @param start_time
   * @return True on success, otherwise False
   */
  bool startRecording(const std::string description, const int id, ros::Time& start_time);

  /*!
   * Start recording data
   * @param description
   * @param start_time
   * @return True on success, otherwise False
   */
  bool startRecording(const task_recorder2_msgs::Description& description, ros::Time& start_time);

  /*!
   * Stop recording data
   * @param start_time
   * @param end_times
   * @param num_samples
   * @param messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const ros::Time& start_time,
                     const ros::Time& end_time,
                     const int num_samples,
                     std::vector<task_recorder2_msgs::DataSample>& messages);

  /*!
   * @param start_time
   * @param end_time
   * @param num_samples
   * @param message_names
   * @param messages
   * @return True on success, otherwise False
   */
  bool stopRecording(const ros::Time& start_time,
                     const ros::Time& end_time,
                     const int num_samples,
                     const std::vector<std::string>& message_names,
                     std::vector<task_recorder2_msgs::DataSample>& messages);

  /*!
   * @return True on success, otherwise False
   */
  bool interruptRecording();

  /*!
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

private:

  /*!
   */
  ros::NodeHandle node_handle_;
  bool is_online_;

  /*!
   */
  ros::ServiceClient start_streaming_service_client_;
  ros::ServiceClient start_recording_service_client_;
  ros::ServiceClient stop_recording_service_client_;
  ros::ServiceClient interrupt_recording_service_client_;
  ros::ServiceClient get_data_sample_service_client_;

};

}

#endif /* TASK_RECORDER_MANAGER_CLIENT_H_ */
