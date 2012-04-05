/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		message_buffer.cpp

  \author	Peter Pastor
  \date		Jun 6, 2011

 *********************************************************************/

// system includes

// local includes
#include <task_recorder2_utilities/message_buffer.h>

namespace task_recorder2_utilities
{

bool MessageBuffer::add(const task_recorder2_msgs::DataSample& data_sample)
{
  // error checking
  if(!data_samples_.empty())
  {
    if(data_samples_[0].data.size() != data_sample.data.size())
    {
      ROS_ERROR("Size of data vector >%i< needs to be >%i<.", (int)data_samples_[0].data.size(), (int)data_sample.data.size());
      return false;
    }
  }

  data_samples_.push_back(data_sample);
  return true;
}


bool MessageBuffer::get(const ros::Time& time, task_recorder2_msgs::DataSample& data_sample)
{
  bool found = false;
  int index = 0;
  for (index = (int)data_samples_.size() - 1; index >= 0 && !found; index--)
  {
    if(data_samples_[index].header.stamp < time)
    {
      data_sample = data_samples_[index];
      found = true;
    }
  }

  if (found && index > 0)
  {
    data_samples_.erase(data_samples_.begin(), data_samples_.begin()+(index-1));
  }
  return found;
}

}
