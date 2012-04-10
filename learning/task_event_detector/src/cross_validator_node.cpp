/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		cross_validator_node.cpp

  \author	Peter Pastor
  \date		Jul 5, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <task_event_detector/cross_validator.h>
#include <task_event_detector/shogun_init.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CrossValidator");
  ros::NodeHandle node_handle("~");
  task_event_detector::init();
  task_event_detector::CrossValidator cross_validator(node_handle);
  cross_validator.run();
  task_event_detector::exit();
  return 0;
}
