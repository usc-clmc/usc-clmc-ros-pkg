/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks http://www.csie.ntu.edu.tw/~cjlin/papers/guide/guide.pdf
 
  \file		cross_validate.cpp

  \author	Peter Pastor
  \date		Jul 5, 2011

 *********************************************************************/

// system includes
#include <Eigen/Eigen>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/logging.h>

#include <omp.h>
// #include <boost/thread.hpp>

// local includes
#include <task_event_detector/cross_validator.h>

using namespace Eigen;

namespace task_event_detector
{

CrossValidator::CrossValidator(ros::NodeHandle node_handle) :
  node_handle_(node_handle), monitor_io_(node_handle)
{
  ROS_VERIFY(readParams());
  // ROS_VERIFY(monitor_io_.initialize(std::string("/TaskRecorderManager/data_samples"), std::string("/TaskRecorderManager/data_samples")));
  ROS_VERIFY(monitor_io_.initialize());
}

void CrossValidator::run()
{
  // read all data
  std::vector<std::vector<task_recorder2_msgs::DataSample> > all_data_samples;
  std::vector<task_recorder2_msgs::DataSampleLabel> all_data_sample_labels;
  ROS_VERIFY(monitor_io_.readAllData(description_, all_data_samples, all_data_sample_labels));
  ROS_ASSERT_MSG(!all_data_samples.empty(), "No data samples read from disc.");
  ROS_ASSERT(all_data_samples.size() == all_data_sample_labels.size());

  std::vector<task_recorder2_msgs::DataSample> data_samples;
  std::vector<task_recorder2_msgs::DataSampleLabel> data_sample_labels;
  for (int i = 0; i < (int)all_data_samples.size(); ++i)
  {
    for (int j = 0; j < (int)all_data_samples[i].size(); ++j)
    {
      data_samples.push_back(all_data_samples[i][j]);
      data_sample_labels.push_back(all_data_sample_labels[i]);
    }
  }
  ROS_ASSERT(data_samples.size() == data_sample_labels.size());
  ROS_INFO("Read total of >%i< labeled data samples read from files...", (int)data_samples.size());

  // prepare data for leave-one-out cross-validation
  std::vector<std::vector<task_recorder2_msgs::DataSample> > cv_data_samples;
  std::vector<std::vector<task_recorder2_msgs::DataSampleLabel> > cv_data_sample_labels;
  for (int i = 0; i < (int)data_samples.size(); i += size_of_validation_set_)
  {
    if (i + size_of_validation_set_ <= (int)data_samples.size())
    {
      std::vector<task_recorder2_msgs::DataSample> samples;
      std::vector<task_recorder2_msgs::DataSampleLabel> labels;
      for (int j = 0; j < size_of_validation_set_; ++j)
      {
        samples.push_back(data_samples[i+j]);
        labels.push_back(data_sample_labels[i+j]);
      }
      cv_data_samples.push_back(samples);
      cv_data_sample_labels.push_back(labels);
    }
  }
  ROS_ASSERT(cv_data_samples.size() == cv_data_sample_labels.size());
  int number_of_validation_sets = (int)cv_data_samples.size();
  ROS_INFO("Created >%i< validation sets.", number_of_validation_sets);

  MatrixXd result = MatrixXd::Zero((DenseIndex)cv_svm_c_.size(), (DenseIndex)cv_svm_width_.size());

  ROS_INFO("Crunching on >%i< samples with >%i< threads.", number_of_validation_sets*size_of_validation_set_, omp_get_max_threads());

  // SVMClassifier::initShogun();
  // let's crunch
#pragma omp parallel for

  for (int i = 0; i < (int)cv_svm_c_.size(); ++i)
  {
    for (int j = 0; j < (int)cv_svm_width_.size(); ++j)
    {
      ROS_DEBUG("Creating new SVM with parameters svm_c >%.6f< and svm_width >%.6f<.", cv_svm_c_[i], cv_svm_width_[j]);

      for (int leave_out_set = 0; leave_out_set < number_of_validation_sets; ++leave_out_set)
      {
        // create new SVM
        SVMClassifier svm_classifier;
        ROS_VERIFY(svm_classifier.read(node_handle_));
        ROS_VERIFY(svm_classifier.setKernelWidth(cv_svm_width_[j]));
        ROS_VERIFY(svm_classifier.setC(cv_svm_c_[i]));
        ROS_VERIFY(svm_classifier.setEps(svm_eps_));

        for (int set = 0; set < number_of_validation_sets; ++set)
        {
          if (set == leave_out_set)
          {
            ROS_DEBUG("Leaving out >%i<.", leave_out_set);
          }
          else
          {
            ROS_DEBUG("Adding >%i< training samples.", (int)cv_data_samples[set].size());
            svm_classifier.addTrainingData(cv_data_samples[set], cv_data_sample_labels[set], detection_variable_names_);
          }
        }

        // train
        ROS_VERIFY(svm_classifier.train());

        // predict
        std::vector<task_recorder2_msgs::DataSampleLabel> test_labels;
        ROS_VERIFY(svm_classifier.predict(cv_data_samples[leave_out_set], test_labels));
        ROS_ASSERT(test_labels.size() == cv_data_sample_labels[leave_out_set].size());

        /*
        for (int r = 0; r < (int)test_labels.size(); ++r)
        {
          ROS_INFO_COND(cv_data_sample_labels[leave_out_set][r].binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED, "CVLabel[%i][%i] = SUCCEEDED.", leave_out_set, r);
          ROS_INFO_COND(cv_data_sample_labels[leave_out_set][r].binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED, "CVLabel[%i][%i] = FAILED.", leave_out_set, r);
          ROS_INFO_COND(cv_data_sample_labels[leave_out_set][r].binary_label.label != task_recorder2_msgs::BinaryLabel::SUCCEEDED
                        && cv_data_sample_labels[leave_out_set][r].binary_label.label != task_recorder2_msgs::BinaryLabel::FAILED, "CVLabel[%i][%i] = UNDEFINED.", leave_out_set, r);

          ROS_INFO_COND(test_labels[r].binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED, "TestLabel[%i] = SUCCEEDED.", r);
          ROS_INFO_COND(test_labels[r].binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED, "TestLabel[%i] = FAILED.", r);
          ROS_INFO_COND(test_labels[r].binary_label.label != task_recorder2_msgs::BinaryLabel::SUCCEEDED
                        && test_labels[r].binary_label.label != task_recorder2_msgs::BinaryLabel::FAILED, "TestLabel[%i] = UNDEFINED (%i).", r, test_labels[r].binary_label.label);
        }
        */

        // evaluate
        for (int n = 0; n < size_of_validation_set_; ++n)
        {
          ROS_ASSERT(test_labels[n].type == cv_data_sample_labels[leave_out_set][n].type);
          if (test_labels[n].binary_label.label != cv_data_sample_labels[leave_out_set][n].binary_label.label)
          {
            result(i, j) += 1.0;
          }
        }
      }
      ROS_INFO("SVM with parameters svm_c >%.6f< and svm_width >%.6f< has >%i< miss classifications.", cv_svm_c_[i], cv_svm_width_[j], (int)result(i,j));
    }
  }

  // SVMClassifier::exitShogun();
  usc_utilities::log(result, "/tmp/result.txt");
}

bool CrossValidator::readParams()
{
  std::string description;
  ROS_VERIFY(usc_utilities::read(node_handle_, "description", description));
  int id;
  ROS_VERIFY(usc_utilities::read(node_handle_, "id", id));
  description_.description = description;
  description_.id = id;

  ROS_VERIFY(usc_utilities::read(node_handle_, "svm_eps", svm_eps_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_c_base", cv_svm_c_base_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_c_min_exp", cv_svm_c_min_exp_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_c_max_exp", cv_svm_c_max_exp_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_c_num_steps", cv_svm_c_num_steps_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_base", cv_svm_width_base_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_min_exp", cv_svm_width_min_exp_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_max_exp", cv_svm_width_max_exp_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_num_steps", cv_svm_width_num_steps_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "exponential_search", exponential_search_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "size_of_validation_set", size_of_validation_set_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "detection_variable_names", detection_variable_names_));

  cv_svm_c_ = getLine(cv_svm_c_min_exp_, cv_svm_c_max_exp_, cv_svm_c_num_steps_, cv_svm_c_base_);
  cv_svm_width_ = getLine(cv_svm_width_min_exp_, cv_svm_width_max_exp_, cv_svm_width_num_steps_, cv_svm_width_base_);

  return true;
}

std::vector<double> CrossValidator::getLine(const double min_exp, const double max_exp, const int num_steps, const double base)
{
  std::vector<double> line;
  ROS_ASSERT(min_exp < max_exp);
  ROS_ASSERT(num_steps >= 2);

  double lin_diff = (max_exp - min_exp) / (num_steps-1);
  for (int i = 0; i < num_steps; ++i)
  {
    double exponent = min_exp + i * lin_diff;
    line.push_back( pow(base, exponent) );
  }

  return line;
}

}
