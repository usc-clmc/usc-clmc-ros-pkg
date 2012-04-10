/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_task_recorder_factory.cpp

  \author	Peter Pastor
  \date		Jun 19, 2011

 *********************************************************************/

// system includes
#include <task_recorder2/joint_states_recorder.h>
#include <task_recorder2/audio_recorder.h>

// local includes
#include <pr2_task_recorder2/pr2_task_recorder_factory.h>

namespace pr2_task_recorder2
{

bool PR2TaskRecorderFactory::createTaskRecorderByName(const std::string& class_name,
                                                      ros::NodeHandle node_handle,
                                                      boost::shared_ptr<task_recorder2::TaskRecorderBase>& task_recorder)
{

  if (class_name == task_recorder2::JointStatesRecorder::getClassName())
  {
    task_recorder.reset(new task_recorder2::JointStatesRecorder(node_handle));
  }
  else if(class_name == task_recorder2::AudioRecorder::getClassName())
  {
    task_recorder.reset(new task_recorder2::AudioRecorder(node_handle));
  }
  else
  {
    ROS_ERROR("Unknown class name: %s.", class_name.c_str());
    return false;
  }

  return true;
}


}
