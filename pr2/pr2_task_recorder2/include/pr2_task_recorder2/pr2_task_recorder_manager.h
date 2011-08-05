/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_task_recorder_manager.h

  \author	Peter Pastor
  \date		Jul 19, 2011

 *********************************************************************/

#ifndef PR2_TASK_RECORDER_MANAGER_H_
#define PR2_TASK_RECORDER_MANAGER_H_

// system includes
#include <vector>
#include <ros/ros.h>

#include <task_recorder2/task_recorder_manager.h>
#include <task_recorder2/task_recorder_base.h>

// local includes

namespace pr2_task_recorder2
{

class PR2TaskRecorderManager : public task_recorder2::TaskRecorderManager
{

public:

  /*! Constructor
   */
  PR2TaskRecorderManager(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~PR2TaskRecorderManager() {};

  /*!
   * @param task_recorders
   * @return
   */
  bool read(std::vector<boost::shared_ptr<task_recorder2::TaskRecorderBase> >& task_recorders);

private:

};


}


#endif /* PR2_TASK_RECORDER_MANAGER_H_ */
