/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		svm_trainer.cpp

  \author	Peter Pastor
  \date		Jul 7, 2011

 *********************************************************************/

// system includes
// system includes
#include <Eigen/Eigen>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/logging.h>

// local includes
#include <task_event_detector/svm_trainer.h>
#include <task_event_detector/svm_classifier.h>
#include <task_event_detector/svm_parameters.h>

using namespace Eigen;

namespace task_event_detector
{

SVMTrainer::SVMTrainer(const task_recorder2_msgs::Description& description,
                       const SVMParametersMsg& parameters) :
    parameters_(parameters), monitor_io_(ros::NodeHandle("/TaskRecorderManager")), description_(description)
{
  ROS_VERIFY(monitor_io_.initialize());
}

bool SVMTrainer::train(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                       const std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels)
{
  if(data_samples.empty() || data_sample_labels.empty()
      || (data_samples.size() != data_sample_labels.size()))
  {
    ROS_ERROR("Input data samples and labels are incorrect.");
    return false;
  }

  SVMClassifierPtr svm_classifier(new SVMClassifier());
  SVMParameters svm_parameters;
  ROS_VERIFY(svm_parameters.set(parameters_));
  ROS_VERIFY(svm_classifier->set(svm_parameters));

  ROS_VERIFY(svm_classifier->setKernelWidth(parameters_.kernel_width));
  ROS_VERIFY(svm_classifier->setC(parameters_.svm_c));
  ROS_VERIFY(svm_classifier->setEps(parameters_.svm_eps));

  ROS_DEBUG("Adding data to SVM.");
  ROS_VERIFY(svm_classifier->addTrainingData(data_samples, data_sample_labels, parameters_.variable_names));

  ROS_DEBUG("Training SVM.");
  ROS_VERIFY(svm_classifier->train());

  ROS_VERIFY(svm_io_.write(description_, svm_classifier));
  return true;
}

}
