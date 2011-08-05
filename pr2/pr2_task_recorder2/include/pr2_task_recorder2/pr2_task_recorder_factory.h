/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_task_recorder_factory.h

  \author	Peter Pastor
  \date		Jun 19, 2011

 *********************************************************************/

#ifndef RP2_TASK_RECORDER_FACTORY_H_
#define PR2_TASK_RECORDER_FACTORY_H_

// system includes
#include <ros/ros.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <task_recorder2/task_recorder_base.h>

// local includes

namespace pr2_task_recorder2
{

class PR2TaskRecorderFactory
{

public:

  static bool createTaskRecorderByName(const std::string& class_name,
                                       ros::NodeHandle node_handle,
                                       boost::shared_ptr<task_recorder2::TaskRecorderBase>& task_recorder);

};

}

#endif /* PR2_TASK_RECORDER_FACTORY_H_ */
