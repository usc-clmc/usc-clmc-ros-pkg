/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_event_detector_node.cpp

  \author	Peter Pastor
  \date		Jun 22, 2011

 *********************************************************************/

// system includes

// local includes
#include <task_event_detector/task_event_detector.h>
#include <task_event_detector/shogun_init.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TaskEventDetector");
  ros::NodeHandle node_handle("~");
  task_event_detector::init();
  task_event_detector::TaskEventDetector task_event_detector(node_handle);
  ros::Duration(1.0).sleep();
  ros::MultiThreadedSpinner mts;
  mts.spin();
  task_event_detector::exit();
  return 0;
}
