/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks       ...

 \file      detector.cpp

 \author    Peter Pastor
 \date      Jun 21, 2011

 *********************************************************************/

// system includes
#include <vector>
#include <boost/shared_ptr.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_utilities/data_sample_label_utilities.h>
#include <task_recorder2_msgs/DetectedEvents.h>

// local includes
#include <task_event_detector/detector.h>
#include <task_event_detector/event_monitor.h>
#include <task_event_detector/data_sample_filter.h>

namespace task_event_detector
{

Detector::Detector() :
    monitor_io_(ros::NodeHandle("/TaskRecorderManager"))
{
  ROS_VERIFY(reset());
}

bool Detector::reset()
{
  svm_classifier_.reset(new SVMClassifier());
  ROS_VERIFY(svm_classifier_->read(ros::NodeHandle("/TaskEventDetector")));
  ROS_VERIFY(monitor_io_.initialize());
  return true;
}

bool Detector::add(const task_recorder2_msgs::Description& description,
                   const std::vector<std::string>& detection_variable_names)
{
  std::vector<task_recorder2_msgs::DataSample> data_samples;
  task_recorder2_msgs::DataSampleLabel data_sample_label;
  ROS_VERIFY(monitor_io_.readData(description, data_samples, data_sample_label));
  ROS_ASSERT_MSG(!data_samples.empty(), "No data samples read from description >%s<.", task_recorder2_utilities::getFileName(description).c_str());
  DataSampleFilterPtr data_sample_filter(new DataSampleFilter());
  ROS_VERIFY(data_sample_filter->initialize(data_samples[0]));
  ROS_VERIFY(data_sample_filter->filter(data_samples));
  ROS_VERIFY(svm_classifier_->addTrainingData(data_samples, data_sample_label, detection_variable_names));
  return true;
}

bool Detector::train()
{
  return svm_classifier_->train();
}

bool Detector::save(task_recorder2_msgs::Description& description)
{
  return svm_io_.write(description, svm_classifier_);
}

bool Detector::load(const task_recorder2_msgs::Description& description)
{
  return svm_io_.read(description, svm_classifier_);
}

bool Detector::filter(std::vector<task_recorder2_msgs::DataSample>& data_samples)
{
  boost::shared_ptr<DataSampleFilter> data_sample_filter(new DataSampleFilter());
  ROS_VERIFY(data_sample_filter->initialize(data_samples.front()));
  for (int i = 0; i < (int)data_samples.size(); ++i)
  {
    ROS_VERIFY(data_sample_filter->filter(data_samples[i]));
  }
  return true;
}

bool Detector::computeLabel(const int method,
                            const std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels,
                            task_recorder2_msgs::DataSampleLabel& data_label)
{
  double value = 0;

  if(data_labels.empty())
  {
    ROS_ERROR("No data labels provided.");
    return false;
  }

  if(method == task_recorder2_msgs::DetectedEvents::Request::AVERAGING)
  {
    double avg = 0;
    for (int i = 0; i < (int)data_labels.size(); ++i)
    {
      avg += static_cast<double>(data_labels[i].cost_label.cost);
    }
    avg /= static_cast<double>(data_labels.size());
    ROS_DEBUG("Averaging over >%i< labels yields >%f<.", (int)data_labels.size(), avg);
    value = avg;
  }
  else if(method == task_recorder2_msgs::DetectedEvents::Request::CHECK_FOR_ONE)
  {

  }
  else
  {
    ROS_ERROR("Unknown method >%i< for computing the label.", method);
    return false;
  }
  return getLabel(value, data_label);
}

bool Detector::predict(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                       std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels)
{
  // error checking
  if (data_samples.empty())
  {
    ROS_ERROR("No data samples provided, cannot predict.");
    return false;
  }
  for (int i = 0; i < (int)data_samples.size(); ++i)
  {
    task_recorder2_msgs::DataSampleLabel data_label;
    ROS_VERIFY(svm_classifier_->predict(data_samples[i], data_label));
    ROS_ASSERT(data_label.type == task_recorder2_msgs::DataSampleLabel::BINARY_LABEL);
    data_labels.push_back(data_label);
  }
  return true;
}

bool Detector::getListOfSVMs(std::vector<std::string>& descriptions)
{
  return svm_io_.getList(descriptions);
}

bool Detector::getListOfDataSamples(std::vector<std::string>& descriptions)
{
  return monitor_io_.getList(descriptions);
}

bool Detector::getListOfRecordedDataSamples(const std::string description_directory,
                                            std::vector<task_recorder2_msgs::Description>& labeled_descriptions,
                                            std::vector<task_recorder2_msgs::Description>& unlabeled_descriptions)
{
  return monitor_io_.getList(description_directory, labeled_descriptions, unlabeled_descriptions);
}

}


