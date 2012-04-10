/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		message_ring_buffer.h

  \author	Peter Pastor
  \date		Jun 6, 2011

 *********************************************************************/

#ifndef MESSAGE_RING_BUFFER_H_
#define MESSAGE_RING_BUFFER_H_

// system includes
#include <vector>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <task_recorder2_msgs/DataSample.h>

// local includes
#include <task_recorder2_utilities/circular_message_buffer.h>

namespace task_recorder2_utilities
{

class MessageRingBuffer
{

  static const int DEFAULT_RING_BUFFER_SIZE = 200;

public:

  /*! Constructor
   */
  MessageRingBuffer(const task_recorder2_msgs::DataSample& default_data_sample,
                    const int ring_buffer_size = DEFAULT_RING_BUFFER_SIZE);
  /*! Destructor
   */
  virtual ~MessageRingBuffer() {};

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

  /*! Constructor must be initialized with default data sample
   */
  MessageRingBuffer();

  /*!
   */
  boost::shared_ptr<CircularMessageBuffer<task_recorder2_msgs::DataSample> > circular_buffer_;

};

}

#endif /* MESSAGE_RING_BUFFER_H_ */
