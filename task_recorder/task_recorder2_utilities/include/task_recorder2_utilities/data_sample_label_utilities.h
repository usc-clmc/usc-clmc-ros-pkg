/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		data_sample_label_utilities.h

  \author	Peter Pastor
  \date		Jun 21, 2011

 *********************************************************************/

#ifndef DATA_SAMPLE_LABEL_UTILITIES_H_
#define DATA_SAMPLE_LABEL_UTILITIES_H_

// system includes
#include <string>
#include <vector>

#include <ros/ros.h>
#include <usc_utilities/assert.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes

namespace task_recorder2_utilities
{

inline bool getSVMLabel(const task_recorder2_msgs::DataSampleLabel& label, double& value)
{
  if(label.type != task_recorder2_msgs::DataSampleLabel::BINARY_LABEL)
  {
    ROS_ERROR("SVM label must be of type >BINARY_LABEL<.");
    return false;
  }

  if(label.binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED)
  {
    value = 1.0;
  }
  else if(label.binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED)
  {
    value = -1.0;
  }
  else
  {
    ROS_ERROR("Binary SVM label is invalid >%i<. Cannot assign binary label. It must be either >%i< or >%i<",
              label.binary_label.label, task_recorder2_msgs::BinaryLabel::SUCCEEDED, task_recorder2_msgs::BinaryLabel::FAILED);
    return false;
  }
  return true;
}

inline bool getBinaryLabel(const double value,
                           task_recorder2_msgs::DataSampleLabel& label,
                           const double classification_boundary)
{
  label.type = task_recorder2_msgs::DataSampleLabel::BINARY_LABEL;
  // if(fabs(value - (-1.0)) < 10e-6)
  if (value > classification_boundary)
  {
    label.binary_label.label = task_recorder2_msgs::BinaryLabel::SUCCEEDED;
  }
  // else if(fabs(value - (1.0)) < 10e-6)
  else
  {
    label.binary_label.label = task_recorder2_msgs::BinaryLabel::FAILED;
  }
  label.cost_label.cost = value;
  //  else
  //  {
  //    ROS_ERROR("Binary SVM label is invalid >%f<. It must be either 1.0 or -1.0.", value);
  //    return false;
  //  }
  return true;
}

}

#endif /* DATA_SAMPLE_LABEL_UTILITIES_H_ */
