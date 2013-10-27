/*
 * interrupt_handler.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: pastor
 */

// local includes
#include <task_recorder2/interrupt_handler.h>

namespace task_recorder2
{

InterruptHandler::InterruptHandler() :
  interrupted_(false)
{
}

void InterruptHandler::resetInterruptHandler()
{
  interrupted_ = false;
  interrupt_start_stamps_.clear();
  interrupt_durations_.clear();
}

void InterruptHandler::interrupt(const ros::Time& now,
                                 const bool is_recording,
                                 const bool recording)
{
  if (is_recording && !recording && !interrupted_)
  {
    interrupted_ = true;
    interrupt_start_stamps_.push_back(now);
    ROS_INFO("Interrupt started at >%f<.", interrupt_start_stamps_.back().toSec());
    ROS_ASSERT(interrupt_start_stamps_.size() - 1 == interrupt_durations_.size());
  }
  else if (!is_recording && recording && interrupted_)
  {
    interrupted_ = false;
    ROS_ASSERT(now > interrupt_start_stamps_.back());
    ROS_INFO("Interrupt finished at >%f<.", now.toSec());
    interrupt_durations_.push_back(now - interrupt_start_stamps_.back());
    ROS_INFO("Interrupt duration was >%.2f< seconds long.", interrupt_durations_.back().toSec());
    ROS_ASSERT(interrupt_start_stamps_.size() == interrupt_durations_.size());
  }
}

void InterruptHandler::getInterrupts(std::vector<ros::Time>& interrupt_start_stamps,
                                     std::vector<ros::Duration>& interrupt_durations) const
{
  interrupt_start_stamps = interrupt_start_stamps_;
  interrupt_durations = interrupt_durations_;
}

}


