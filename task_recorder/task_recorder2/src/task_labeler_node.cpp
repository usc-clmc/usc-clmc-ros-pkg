/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_labeler_node.cpp

  \author	Peter Pastor
  \date		Jun 17, 2011

 *********************************************************************/

// system includes

#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes
#include <task_recorder2/task_labeler.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TaskLabeler");
  ros::NodeHandle node_handle("~");

  task_recorder2::TaskLabeler<task_recorder2_msgs::DataSampleLabel> data_sample_labeler(node_handle);

  ros::spin();
  return 1;
}
