/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_monitor.h

  \author	Peter Pastor
  \date		Jun 8, 2011

 *********************************************************************/

#ifndef TASK_MONITOR_H_
#define TASK_MONITOR_H_

// system includes
#include <ros/ros.h>
#include <string>
#include <vector>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes

namespace task_monitor
{

class TaskMonitor
{

public:

  /*! Constructor
   */
  TaskMonitor(ros::NodeHandle node_handle, const std::string topic_name = "/TaskRecorderManager/data_sample");
  /*! Destructor
   */
  virtual ~TaskMonitor() {};

  /*!
   */
  bool initialize();

private:

  /*!
   */
  ros::NodeHandle node_handle_;
  ros::Subscriber data_sample_subcriber_;



  /*!
   */
  std::vector<task_recorder2_msgs::DataSample> data_samples_;

  /*!
   * @param data_sample
   */
  void monitorCallback(const task_recorder2_msgs::DataSampleConstPtr data_sample);

};

}

#endif /* TASK_MONITOR_H_ */
