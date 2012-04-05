/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		event_monitor.cpp

  \author	Peter Pastor
  \date		Aug 30, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>

// local includes
#include <task_event_detector/event_monitor.h>

namespace task_event_detector
{

EventMonitor::EventMonitor() :
  first_time_(true)
{
  data_sample_filter_.reset(new DataSampleFilter());
  const int SUBSCRIBER_BUFFER_SIZE = 10;
  data_sample_subscriber_ = node_handle_.subscribe("/TaskRecorderManager/data_samples", SUBSCRIBER_BUFFER_SIZE, &EventMonitor::dataSampleCB, this);
}

void EventMonitor::registerCallback(boost::function<void (const task_recorder2_msgs::DataSample& data_sample)> callback)
{
  boost::mutex::scoped_lock lock(callback_mutex_);
  user_callbacks_.push_back(callback);
}

void EventMonitor::unregisterCallback(boost::function<void (const task_recorder2_msgs::DataSample& data_sample)> callback)
{
//  boost::mutex::scoped_lock lock(callback_mutex_);
//  std::vector<boost::function<void(const task_recorder2_msgs::DataSample& data_sample)> >::iterator f = find(user_callbacks_.begin(), user_callbacks_.end(), callback);
//  if (f != user_callbacks_.end())
//  {
//    user_callbacks_.erase(f);
//  }
}

void EventMonitor::dataSampleCB(const task_recorder2_msgs::DataSample::ConstPtr& msg)
{
  task_recorder2_msgs::DataSample data_sample = *msg;
  if(first_time_)
  {
    ROS_VERIFY(data_sample_filter_->initialize(data_sample));
    first_time_ = false;
  }
  ROS_VERIFY(data_sample_filter_->filter(data_sample));
  boost::mutex::scoped_lock lock(callback_mutex_);
  for (int i = 0; i < (int)user_callbacks_.size(); ++i)
  {
    user_callbacks_[i](data_sample);
  }
}

}
