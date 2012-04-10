/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_accumulator.h

  \author	Peter Pastor
  \date		Jun 8, 2011

 *********************************************************************/

#ifndef TASK_ACCUMULATOR_H_
#define TASK_ACCUMULATOR_H_

// system includes
#include <ros/ros.h>
#include <string>
#include <vector>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

#include <task_recorder2_utilities/task_accumulator_io.h>

// local includes
#include <task_recorder2_msgs/Accumulate.h>

namespace task_monitor
{

class TaskAccumulator
{

public:

  /*! Constructor
   */
  TaskAccumulator(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~TaskAccumulator() {};

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  bool accumulate(task_recorder2_msgs::Accumulate::Request& request,
                  task_recorder2_msgs::Accumulate::Response& response);

private:

  /*!
   */
  ros::ServiceServer accumulator_service_;

  /*!
   */
  task_recorder2_utilities::TaskAccumulatorIO<> accumulator_io_;

  std::string topic_name_;

};

}

#endif /* TASK_ACCUMULATOR_H_ */
