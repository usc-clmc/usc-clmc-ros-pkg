/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks http://www.csie.ntu.edu.tw/~cjlin/papers/guide/guide.pdf
 
  \file		svm_classifier.h

  \author	Peter Pastor
  \date		Jun 21, 2011

 *********************************************************************/

#ifndef SVM_CLASSIFIER_H_
#define SVM_CLASSIFIER_H_

// system includes
#include <vector>
#include <fcntl.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <shogun/kernel/DotKernel.h>
#include <shogun/classifier/svm/SVM.h>
#include <shogun/features/SimpleFeatures.h>

#include <shogun/lib/common.h>
#include <shogun/base/init.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes
#include <task_event_detector/svm_parameters.h>

namespace task_event_detector
{

static const bool SVM_LOGGING_ENABLED = true;

class SVMClassifier
{

public:

  static const int MAX_NUM_TEST_SAMPLES = 100;
  static const int MAX_NUM_TEST_DIMENSIONS = 30;

  /*! Constructor
   */
  SVMClassifier();
  /*! Destructor
   */
  virtual ~SVMClassifier();

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool read(ros::NodeHandle node_handle);

  /*!
   * @param svm_parameters
   * @return True on success, otherwise False
   */
  bool set(const SVMParameters& svm_parameters);

  /*!
   * @param kernel_width
   * @return True on success, otherwise False
   */
  bool setKernelWidth(const double kernel_width)
  {
    ROS_ASSERT(svm_parameters_.initialized_);
    svm_parameters_.msg_.kernel_width = kernel_width;
    return true;
  }

  /*!
   * @param svm_c
   * @return True on success, otherwise False
   */
  bool setC(const double svm_c)
  {
    ROS_ASSERT(svm_parameters_.initialized_);
    svm_parameters_.msg_.svm_c = svm_c;
    return true;
  }

  /*!
   * @param svm_eps
   * @return True on success, otherwise False
   */
  bool setEps(const double svm_eps)
  {
    ROS_ASSERT(svm_parameters_.initialized_);
    svm_parameters_.msg_.svm_eps = svm_eps;
    return true;
  }

  /*!
   * @param directory_name
   * @return True on success, otherwise false
   */
  bool load(const std::string directory_name = std::string("/tmp"));

  /*!
   * @param directory_name
   * @return True on success, otherwise false
   */
  bool save(const std::string directory_name = std::string("/tmp"));

  /*!
   * @return True if initialized, otherwise False
   */
  bool isInitialized() const
  {
    return svm_parameters_.initialized_;
  }
  /*!
   * @return True if trained, otherwise False
   */
  bool isTrained() const
  {
    return (trained_ || loaded_);
  }

  /*!
   * @param data_sample
   * @param data_label
   * @param variable_names
   * @return True on success, otherwise False
   */
  bool addTrainingData(const task_recorder2_msgs::DataSample& data_sample,
                       const task_recorder2_msgs::DataSampleLabel& data_label,
                       const std::vector<std::string> variable_names = std::vector<std::string>());

  /*!
   * @param data_samples
   * @param data_label
   * @param variable_names
   * @return True on success, otherwise False
   */
  bool addTrainingData(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                       const task_recorder2_msgs::DataSampleLabel& data_label,
                       const std::vector<std::string> variable_names = std::vector<std::string>());

  /*!
   * @param data_samples
   * @param data_labels
   * @param variable_names
   * @return True on success, otherwise False
   */
  bool addTrainingData(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                       const std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels,
                       const std::vector<std::string> variable_names = std::vector<std::string>());

  /*!
   * @return True on success, otherwise False
   */
  bool train();

  /*!
   * @param data_sample
   * @param data_label
   // * @param variable_names
   * @return True on success, otherwise False
   */
  bool predict(const task_recorder2_msgs::DataSample& data_sample,
               task_recorder2_msgs::DataSampleLabel& data_label); /*,
               const std::vector<std::string> variable_names = std::vector<std::string>());*/

  /*!
   * @param data_samples
   * @param data_label
   // * @param variable_names
   * @return True on success, otherwise False
   */
  bool predict(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
               std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels);/*,
               const std::vector<std::string> variable_names = std::vector<std::string>());*/

  /*!
   * @param value
   * @param label
   * @return True on success, otherwise False
   */
  bool getLabel(const double value, task_recorder2_msgs::DataSampleLabel& label);

  /*!
   * @param classification_boundary
   * @return True on success, otherwise False
   */
  bool setClassificationBoundary(const double classification_boundary);

  /*!
   * @return
   */
  double getClassificationBoundary() const;

  /*!
   * @param msg
   * @return True on success, otherwise False
   */
  bool setSVMParametersMsg(const SVMParametersMsg& msg);

  /*!
   * @param msg
   * @return True on success, otherwise False
   */
  bool getSVMParametersMsg(SVMParametersMsg& msg) const;

private:

  bool trained_;
  bool loaded_;

  std::vector<task_recorder2_msgs::DataSample> data_samples_;
  std::vector<task_recorder2_msgs::DataSampleLabel> data_labels_;

  std::vector<std::vector<double> > learned_feature_matrix_;

  float64_t* training_features_;
  float64_t* training_labels_;
  float64_t* test_features_;
  float64_t* test_labels_;

  shogun::CLabels* ctraining_labels_;
  shogun::CSimpleFeatures<float64_t>* ctraining_features_;
  shogun::CLabels* ctest_labels_;
  shogun::CSimpleFeatures<float64_t>* ctest_features_;
  shogun::CDotKernel* ckernel_;
  shogun::CSVM* svm_;

  SVMParameters svm_parameters_;

  void reset();
  void freeSVM();
  void clear();

  bool createKernel();
  bool createSVM();

  /*!
   * @return True on success, otherwise False
   */
  bool initialize();

};

/*! Abbreviatons for convinience
 */
typedef boost::shared_ptr<SVMClassifier> SVMClassifierPtr;
typedef boost::shared_ptr<SVMClassifier const> SVMClassifierConstPtr;

}

#endif /* SVM_CLASSIFIER_H_ */
