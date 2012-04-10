/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		svm_trainer_node.cpp

  \author	Peter Pastor
  \date		Jun 22, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_msgs/Description.h>

// local includes
#include <task_event_detector/svm_trainer.h>
#include <task_event_detector/shogun_init.h>
#include <task_event_detector/svm_parameters.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SVMTrainer");
  ros::NodeHandle node_handle("~");
  task_event_detector::init();

  task_event_detector::SVMParameters svm_parameters;
  ROS_VERIFY(svm_parameters.read(node_handle));

  task_recorder2_msgs::Description description;
  ROS_VERIFY(usc_utilities::read(node_handle, "description", description.description));
  ROS_VERIFY(usc_utilities::read(node_handle, "id", description.id));

  task_event_detector::SVMTrainer svm_trainer(description, svm_parameters.get());
  ROS_ERROR("THIS NEED TO BE FIXED... AFTER THE TEST.");
  // svm_trainer.train();
  task_event_detector::exit();
  return 0;
}
