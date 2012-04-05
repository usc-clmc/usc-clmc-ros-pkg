/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		event_monitor.h

  \author	Peter Pastor
  \date		Aug 30, 2011

 *********************************************************************/

#ifndef EVENT_MONITOR_H_
#define EVENT_MONITOR_H_

// system includes
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSample.h>

// local includes
#include <task_event_detector/data_sample_filter.h>

namespace task_event_detector
{

class EventMonitor
{

public:

  EventMonitor();
  virtual ~EventMonitor() {};

  /**
   * Register a callback function which will be called every time data samples are updated
   * @param callback
   */
  void registerCallback(boost::function<void (const task_recorder2_msgs::DataSample& data_sample)> callback);
  /**
   * Unregister a callback function which will be called every time data samples are updated
   * @param callback
   */
  void unregisterCallback(boost::function<void (const task_recorder2_msgs::DataSample& data_sample)> callback);

private:

  ros::NodeHandle node_handle_;

  boost::mutex callback_mutex_;
  std::vector<boost::function<void (const task_recorder2_msgs::DataSample& data_sample)> > user_callbacks_;

  ros::Subscriber data_sample_subscriber_;
  void dataSampleCB(const task_recorder2_msgs::DataSample::ConstPtr& msg);

  /*! One for all
   */
  bool first_time_;
  DataSampleFilterPtr data_sample_filter_;

};

}

#endif /* EVENT_MONITOR_H_ */
