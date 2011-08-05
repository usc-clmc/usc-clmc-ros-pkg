/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		message_ring_buffer.cpp

  \author	Peter Pastor
  \date		Jun 6, 2011

 *********************************************************************/

// system includes

// local includes
#include <task_recorder2_utilities/message_ring_buffer.h>
#include <boost/circular_buffer.hpp>

namespace task_recorder2_utilities
{

MessageRingBuffer::MessageRingBuffer(const task_recorder2_msgs::DataSample& default_data_sample,
                                     const int ring_buffer_size)
{
  circular_buffer_.reset(new CircularMessageBuffer<task_recorder2_msgs::DataSample>(ring_buffer_size, default_data_sample));
}

bool MessageRingBuffer::add(const task_recorder2_msgs::DataSample& data_sample)
{
  // error checking
  if(!circular_buffer_->empty())
  {
    if(circular_buffer_->front().data.size() != data_sample.data.size())
    {
      ROS_ERROR("Size of data vector >%i< needs to be >%i<.",
                (int)circular_buffer_->front().data.size(), (int)data_sample.data.size());
      return false;
    }
  }

  // fill the buffer if it has been empty
  if(circular_buffer_->size() == 0)
  {
    for (unsigned int i = 0; i < circular_buffer_->capacity(); ++i)
    {
      circular_buffer_->push_back(data_sample);
    }
  }

  circular_buffer_->push_back(data_sample);
  return true;
}

bool MessageRingBuffer::get(const ros::Time& time, task_recorder2_msgs::DataSample& data_sample)
{
  bool found = false;
  boost::circular_buffer<task_recorder2_msgs::DataSample>::const_reverse_iterator rci;
  for(rci = circular_buffer_->cb_.rbegin(); rci!= circular_buffer_->cb_.rend() && !found; ++rci)
  {
    if (rci->header.stamp <= time)
    {
      data_sample = *rci;
      found = true;
    }
  }
  return found;
}

}
