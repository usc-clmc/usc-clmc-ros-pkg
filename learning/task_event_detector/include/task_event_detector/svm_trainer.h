/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		svm_trainer.h

  \author	Peter Pastor
  \date		Jul 7, 2011

 *********************************************************************/

#ifndef SVM_TRAINER_H_
#define SVM_TRAINER_H_

// system includes
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

#include <task_recorder2_utilities/task_monitor_io.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_msgs/Description.h>

// local includes
#include <task_event_detector/svm_io.h>

namespace task_event_detector
{

class SVMTrainer
{

public:

  /*! Constructor
   */
  SVMTrainer(const task_recorder2_msgs::Description& description,
             const SVMParametersMsg& parameters);

  /*! Destructor
   */
  virtual ~SVMTrainer() {};

  /*!
   */
  bool train(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
             const std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels);

private:

  task_event_detector::SVMParametersMsg parameters_;
  task_event_detector::SVMIO svm_io_;
  task_recorder2_utilities::TaskMonitorIO<task_recorder2_msgs::DataSample, task_recorder2_msgs::DataSampleLabel> monitor_io_;
  task_recorder2_msgs::Description description_;

};

}

#endif /* SVM_TRAINER_H_ */
