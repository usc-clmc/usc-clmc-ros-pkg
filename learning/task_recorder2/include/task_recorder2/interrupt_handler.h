/*
 * interrupt_handler.h
 *
 *  Created on: Oct 26, 2013
 *      Author: pastor
 */

#ifndef INTERRUPT_HANDLER_H_
#define INTERRUPT_HANDLER_H_

// system includes
#include <ros/ros.h>
#include <vector>

namespace task_recorder2
{

class InterruptHandler
{
public:

  InterruptHandler();
  virtual ~InterruptHandler() {};

protected:

  void resetInterruptHandler();
  void interrupt(const ros::Time& now,
                 const bool is_recording,
                 const bool recording);

  void getInterrupts(std::vector<ros::Time>& interrupt_start_stamps,
                     std::vector<ros::Duration>& interrupt_durations) const;

private:
  bool interrupted_;
  std::vector<ros::Time> interrupt_start_stamps_;
  std::vector<ros::Duration> interrupt_durations_;

};

}

#endif /* INTERRUPT_HANDLER_H_ */
