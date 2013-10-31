/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks              ...
 
  \file         task_recorder_base.h

  \author       Peter Pastor
  \date         Jun 19, 2011

 *********************************************************************/

#ifndef TASK_RECORDER_BASE_H_
#define TASK_RECORDER_BASE_H_

// system includes
#include <ros/ros.h>
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/TaskRecorderSpecification.h>

#include <string>
#include <boost/shared_ptr.hpp>

#include <task_recorder2_srvs/StartStreaming.h>
#include <task_recorder2_srvs/StopStreaming.h>
#include <task_recorder2_srvs/StartRecording.h>
#include <task_recorder2_srvs/StopRecording.h>
#include <task_recorder2_srvs/InterruptRecording.h>

// local includes
#include <task_recorder2_msgs/definitions.h>

namespace task_recorder2
{

class TaskRecorderBase
{

public:

  /*! Constructor
   */
  TaskRecorderBase() {};
  /*! Destructor
   */
  virtual ~TaskRecorderBase() {};

  /*!
   * @param task_recorder_specification
   * @return True on success, otherwise False
   */
  virtual bool initialize(const task_recorder2_msgs::TaskRecorderSpecification& task_recorder_specification) = 0;

  /*!
   * @param node_handle
   * @param class_name
   * @param class_name_prefix
   * @return True on success, otherwise False
   */
  virtual bool readParams(ros::NodeHandle& node_handle,
                          const std::string& class_name,
                          const std::string& class_name_prefix) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool startStreaming(task_recorder2_srvs::StartStreaming::Request& request,
                              task_recorder2_srvs::StartStreaming::Response& response) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool stopStreaming(task_recorder2_srvs::StopStreaming::Request& request,
                             task_recorder2_srvs::StopStreaming::Response& response) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool startRecording(task_recorder2_srvs::StartRecording::Request& request,
                              task_recorder2_srvs::StartRecording::Response& response) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool stopRecording(task_recorder2_srvs::StopRecording::Request& request,
                             task_recorder2_srvs::StopRecording::Response& response) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool interruptRecording(task_recorder2_srvs::InterruptRecording::Request& request,
                                  task_recorder2_srvs::InterruptRecording::Response& response) = 0;

  /*!
   * @param first
   * @param last
   * @return True if at least one sample has been recorded, meaning that
   * the first/last time stamps are actually meaningful, otherwise False
   */
  virtual bool getTimeStamps(ros::Time& first, ros::Time& last) = 0;
  /*!
   * @param first
   * @param last
   * @param is_recording
   * @param is_streaming
   * @param num_recorded_messages
   * @return True if at least one sample has been recorded, meaning that
   * the first/last time stamps are actually meaningful, otherwise False
   */
  virtual bool getTimeStamps(ros::Time& first, ros::Time& last,
                             bool& is_recording, bool& is_streaming,
                             unsigned int& num_recorded_messages) = 0;

  /*!
   * @param time
   * @param data_sample
   * @return True on success, otherwise False
   */
  virtual bool getSampleData(const ros::Time& time,
                             task_recorder2_msgs::DataSample& data_sample) = 0;

  /*!
   * @return All the variable names that the task recorder can record including prefix
   */
  virtual std::vector<std::string> getPrefixedNames() const = 0;

  /*!
   * @return
   */
  static std::string getClassName();

  /*!
   * This function will be called right before each recording is started
   * @return True on success, otherwise False
   */
  virtual bool startRecording() = 0;

  /*!
   * @return
   */
  virtual bool usesTimer() const = 0;

protected:

private:

};

typedef boost::shared_ptr<TaskRecorderBase> TaskRecorderPtr;

}

#endif /* TASK_RECORDER_BASE_H_ */
