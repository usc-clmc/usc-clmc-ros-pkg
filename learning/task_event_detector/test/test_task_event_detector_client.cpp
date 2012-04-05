/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		test_task_event_detector_client.cpp

  \author	Peter Pastor
  \date		Jun 21, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/logging.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes
#include <task_event_detector/task_event_detector_client.h>
#include <task_event_detector/shogun_init.h>

using namespace task_event_detector;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestTaskEventDetectorClient");
  ros::NodeHandle node_handle("~");
  task_event_detector::init();

  TaskEventDetectorClient task_event_detector_client;

  task_recorder2_msgs::Description description;
  description.description.assign("test_description");
  description.id = 0;

  ROS_INFO_COND(task_event_detector_client.doesSVMExist(description), "SVM >%s< does exist.", task_recorder2_utilities::getFileName(description).c_str());
  ROS_INFO_COND(!task_event_detector_client.doesSVMExist(description), "SVM >%s< does not exist.", task_recorder2_utilities::getFileName(description).c_str());

  ROS_VERIFY(task_event_detector_client.labelSample("test_description"));
  task_event_detector::exit();
  return 0;
}
