/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks http://www.csie.ntu.edu.tw/~cjlin/papers/guide/guide.pdf
 
  \file		cross_validator.h

  \author	Peter Pastor
  \date		Jul 5, 2011

 *********************************************************************/

#ifndef CROSS_VALIDATOR_H_
#define CROSS_VALIDATOR_H_

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
#include <task_event_detector/svm_classifier.h>

namespace task_event_detector
{

class CrossValidator
{

public:

  /*! Constructor
   */
  CrossValidator(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~CrossValidator() {};

  /*!
   */
  void run();

private:

  ros::NodeHandle node_handle_;

  bool readParams();
  double svm_eps_;

  double cv_svm_c_base_;
  double cv_svm_c_min_exp_;
  double cv_svm_c_max_exp_;
  int cv_svm_c_num_steps_;
  std::vector<double> cv_svm_c_;

  double cv_svm_width_base_;
  double cv_svm_width_min_exp_;
  double cv_svm_width_max_exp_;
  int cv_svm_width_num_steps_;
  std::vector<double> cv_svm_width_;

  std::vector<std::string> detection_variable_names_;

  bool exponential_search_;
  int size_of_validation_set_;

  task_recorder2_utilities::TaskMonitorIO<task_recorder2_msgs::DataSample, task_recorder2_msgs::DataSampleLabel> monitor_io_;
  task_recorder2_msgs::Description description_;

  std::vector<double> getLine(const double min, const double max, const int num_steps, const double base);

};

}

#endif /* CROSS_VALIDATOR_H_ */
