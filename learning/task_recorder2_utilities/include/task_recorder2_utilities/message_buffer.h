/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		message_buffer.h

  \author	Peter Pastor
  \date		Jun 6, 2011

 *********************************************************************/

#ifndef MESSAGE_BUFFER_H_
#define MESSAGE_BUFFER_H_

// system includes
#include <vector>
#include <ros/ros.h>

// local includes
#include <task_recorder2_msgs/DataSample.h>

namespace task_recorder2_utilities
{

class MessageBuffer
{

public:

  /*! Constructor
   */
  MessageBuffer() {};
  /*! Destructor
   */
  virtual ~MessageBuffer() {};

  /*!
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool add(const task_recorder2_msgs::DataSample& data_sample);

  /*!
   * @param time
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool get(const ros::Time& time, task_recorder2_msgs::DataSample& data_sample);

private:

  std::vector<task_recorder2_msgs::DataSample> data_samples_;

};

}

#endif /* MESSAGE_BUFFER_H_ */
