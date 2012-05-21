/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_recorder_base.h

  \author	Peter Pastor
  \date		Jun 19, 2011

 *********************************************************************/

#ifndef TASK_RECORDER_BASE_H_
#define TASK_RECORDER_BASE_H_

// system includes
#include <ros/ros.h>
#include <task_recorder2_msgs/DataSample.h>

#include <string>
#include <boost/shared_ptr.hpp>

// local includes
#include <task_recorder2/StartStreaming.h>
#include <task_recorder2/StartRecording.h>
#include <task_recorder2/StopRecording.h>
#include <task_recorder2/InterruptRecording.h>

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
   * @param topic_name
   * @param service_prefix
   * @param splining_method
   * @return True on success, otherwise False
   */
  virtual bool initialize(const std::string topic_name,
                          const std::string service_prefix,
                          const std::string splining_method) = 0;

  /*!
   * @param node_handle The nodehandle that is specific to the (particular) task recorder
   * Derived classes can implement this function to retrieve (arm) specific parameters
   * @return True on success, otherwise False
   */
  virtual bool readParams(ros::NodeHandle& node_handle) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool startStreaming(task_recorder2::StartStreaming::Request& request,
                              task_recorder2::StartStreaming::Response& response) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool startRecording(task_recorder2::StartRecording::Request& request,
                              task_recorder2::StartRecording::Response& response) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool stopRecording(task_recorder2::StopRecording::Request& request,
                             task_recorder2::StopRecording::Response& response) = 0;

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  virtual bool interruptRecording(task_recorder2::InterruptRecording::Request& request,
                                  task_recorder2::InterruptRecording::Response& response) = 0;

  /*!
   * @param first
   * @param last
   * @return True if successful (i.e. recorder is currently recording), otherwise False
   */
  virtual bool getTimeStamps(ros::Time& first,
                             ros::Time& last) = 0;

  /*!
   * @param time
   * @param data_sample
   * @return True on success, otherwise False
   */
  virtual bool getSampleData(const ros::Time& time,
                             task_recorder2_msgs::DataSample& data_sample) = 0;

  /*!
   * @return All the variable names that the task recorder can record
   */
  virtual std::vector<std::string> getNames() const = 0;

  /*!
   * @return
   */
  static std::string getClassName();

  /*!
   * This function will be called right before each recording is started
   * @return True on success, otherwise False
   */
  virtual bool startRecording() = 0;

private:

};

typedef boost::shared_ptr<TaskRecorderBase> TaskRecorderPtr;

}

#endif /* TASK_RECORDER_BASE_H_ */
