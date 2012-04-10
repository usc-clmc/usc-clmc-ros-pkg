/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks       ...

 \file      detector.h

 \author    Peter Pastor
 \date      Jun 21, 2011

 *********************************************************************/

#ifndef DETECTOR_H_
#define DETECTOR_H_

// system includes
#include <ros/ros.h>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

#include <task_recorder2_utilities/task_monitor_io.h>

// local includes
#include <task_event_detector/svm_classifier.h>
#include <task_event_detector/svm_io.h>
#include <task_event_detector/SVMParametersMsg.h>

namespace task_event_detector
{

class Detector
{

public:

  Detector();
  virtual ~Detector() {};

  /*!
   * @return True on success, otherwise False
   */
  bool reset();

  /*!
   * @param description
   * @param detection_variable_names
   * @return True on success, otherwise False
   */
  bool add(const task_recorder2_msgs::Description& description,
           const std::vector<std::string>& detection_variable_names);

  /*!
   * @return True on success, otherwise False
   */
  bool train();

  /*!
   * @param data_samples
   * @return True on success, otherwise False
   */
  bool filter(std::vector<task_recorder2_msgs::DataSample>& data_samples);

  /*!
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool filter(task_recorder2_msgs::DataSample& data_sample)
  {
    std::vector<task_recorder2_msgs::DataSample> data_samples;
    data_samples.push_back(data_sample);
    if(!filter(data_samples))
    {
      return false;
    }
    data_sample = data_samples[0];
    return true;
  }

  /*!
   * @param data_samples
   * @param data_labels
   * @return True on success, otherwise False
   */
  bool predict(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
               std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels);

  /*!
   * @param data_sample
   * @param data_label
   * @return True on success, otherwise False
   */
  bool predict(const task_recorder2_msgs::DataSample& data_sample,
               task_recorder2_msgs::DataSampleLabel& data_label)
  {
    std::vector<task_recorder2_msgs::DataSample> data_samples;
    data_samples.push_back(data_sample);
    std::vector<task_recorder2_msgs::DataSampleLabel> data_labels;
    if(!predict(data_samples, data_labels))
    {
      return false;
    }
    ROS_ASSERT_MSG(data_labels.size() == 1, "Number of received labels MUST be 1. This should never happen.");
    data_label = data_labels[0];
    return true;
  }

  /*!
   * @param descriptions
   * @return True on success, otherwise False
   */
  bool getListOfSVMs(std::vector<std::string>& descriptions);

  /*!
   * @param descriptions
   * @return True on success, otherwise False
   */
  bool getListOfDataSamples(std::vector<std::string>& descriptions);

  /*!
   * @param description_directory
   * @param labeled_descriptions
   * @param unlabeled_descriptions
   * @return True on success, otherwise False
   */
  bool getListOfRecordedDataSamples(const std::string description_directory,
                                    std::vector<task_recorder2_msgs::Description>& labeled_descriptions,
                                    std::vector<task_recorder2_msgs::Description>& unlabeled_descriptions);

  /*!
   * @return True if SVM is trained, False otherwise
   */
  bool isTrained() const
  {
    return svm_classifier_->isTrained();
  }

  /*!
   * @param description
   * @return True on success, otherwise False
   */
  bool save(task_recorder2_msgs::Description& description);

  /*!
   * @param description
   * @return True on success, otherwise False
   */
  bool load(const task_recorder2_msgs::Description& description);

  /*!
   * @param classification_boundary
   * @return
   */
  bool setClassificationBoundary(const double classification_boundary)
  {
    return svm_classifier_->setClassificationBoundary(classification_boundary);
  }

  /*!
   * @return
   */
  double getClassificationBoundary() const
  {
    return svm_classifier_->getClassificationBoundary();
  }

  /*!
   * @param msg
   * @return True on success, otherwise False
   */
  bool setSVMParametersMsg(const SVMParametersMsg& msg)
  {
    return svm_classifier_->setSVMParametersMsg(msg);
  }

  /*!
   * @param msg
   * @return True on success, otherwise False
   */
  bool getSVMParametersMsg(SVMParametersMsg& msg) const
  {
    return svm_classifier_->getSVMParametersMsg(msg);
  }

protected:

  /*!
   * @param method
   * @param data_label
   * @return True on success, otherwise False
   */
  bool computeLabel(const int method,
                    const std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels,
                    task_recorder2_msgs::DataSampleLabel& data_label);

  /*!
   * @param value
   * @param label
   * @return True on success, otherwise False
   */
  bool getLabel(const double value,
                task_recorder2_msgs::DataSampleLabel& label)
  {
    return svm_classifier_->getLabel(value, label);
  }

private:

  /*!
   */
  SVMIO svm_io_;
  SVMClassifierPtr svm_classifier_;


  /*!
   */
  task_recorder2_utilities::TaskMonitorIO<task_recorder2_msgs::DataSample, task_recorder2_msgs::DataSampleLabel> monitor_io_;

};

}


#endif /* DETECTOR_H_ */
