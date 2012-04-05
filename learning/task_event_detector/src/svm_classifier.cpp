/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks http://www.csie.ntu.edu.tw/~cjlin/papers/guide/guide.pdf

 \file   svm_classifier.cpp

 \author Peter Pastor
 \date   Jun 21, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <shogun/io/AsciiFile.h>
#include <shogun/io/SerializableAsciiFile.h>

#include <shogun/classifier/svm/SVMLight.h>
#include <shogun/classifier/svm/SVMLightOneClass.h>
#include <shogun/classifier/svm/LibSVM.h>
#include <shogun/classifier/svm/LibSVMOneClass.h>

#include <stdlib.h>
#include <stdio.h>

#include <task_recorder2_utilities/data_sample_utilities.h>
#include <task_recorder2_utilities/data_sample_label_utilities.h>

#include <shogun/kernel/GaussianKernel.h>
#include <shogun/kernel/LinearKernel.h>

#include <usc_utilities/logging.h>

// local includes
#include <task_event_detector/svm_classifier.h>

namespace task_event_detector
{

const int FEATURE_CACHE_SIZE = 0;
const int KERNEL_CACHE_SIZE = 0;

static const std::string SVM_FILE_NAME = "svm.txt";
static const std::string KERNEL_FILE_NAME = "svm_kernel.txt";
static const std::string LABELS_FILE_NAME = "svm_labels.txt";
// static const std::string FEATURES_FILE_NAME = "svm_features.txt";
// static const std::string SVM_PARAMETERS_FILE_NAME = "svm_parameters.txt";

SVMClassifier::SVMClassifier() :
  trained_(false), loaded_(false)
{
  ROS_VERIFY(initialize());
}

SVMClassifier::~SVMClassifier()
{
  clear();
}

bool SVMClassifier::read(ros::NodeHandle node_handle)
{
  ROS_VERIFY(svm_parameters_.read(node_handle));
  return svm_parameters_.initialized_;
}

bool SVMClassifier::set(const SVMParameters& svm_parameters)
{
  svm_parameters_ = svm_parameters;
  return svm_parameters_.initialized_;
}

bool SVMClassifier::initialize()
{
  clear();
  test_labels_ = new float64_t[MAX_NUM_TEST_SAMPLES];
  test_features_ = new float64_t[MAX_NUM_TEST_SAMPLES * MAX_NUM_TEST_DIMENSIONS];
  ctest_labels_ = new shogun::CLabels();
  SG_REF(ctest_labels_);
  ctest_features_ = new shogun::CSimpleFeatures<float64_t>(FEATURE_CACHE_SIZE);
  SG_REF(ctest_features_);
  return true;
}

void SVMClassifier::reset()
{
  ROS_ASSERT(svm_parameters_.initialized_);
  svm_parameters_.msg_.indices.clear();
  svm_parameters_.msg_.num_variables = 0;
  svm_parameters_.msg_.num_data_samples = 0;
  data_samples_.clear();
  data_labels_.clear();
}

void SVMClassifier::freeSVM()
{
  if(trained_)
  {
    // TODO: fix this "double free" seg fault
    // SG_UNREF(ctraining_features_);
    // SG_UNREF(ctraining_labels_);
  }
  if (trained_ || loaded_)
  {
    SG_UNREF(ctest_labels_);
    SG_UNREF(ctest_features_);
    SG_UNREF(ckernel_);
    SG_UNREF(svm_);
    loaded_ = false;
    trained_ = false;
  }
}

void SVMClassifier::clear()
{
  if (svm_parameters_.initialized_)
  {
    ROS_DEBUG("Deleting test labels and features.");
    delete[] test_labels_;
    delete[] test_features_;
  }
  if (trained_)
  {
    if(svm_parameters_.msg_.num_data_samples > 0)
    {
      ROS_DEBUG("Deleting training labels and features.");
      delete[] training_features_;
      delete[] training_labels_;
    }
    freeSVM();
  }
}

bool SVMClassifier::addTrainingData(const task_recorder2_msgs::DataSample& data_sample,
                                    const task_recorder2_msgs::DataSampleLabel& data_label,
                                    const std::vector<std::string> variable_names)
{
  std::vector<task_recorder2_msgs::DataSample> data_samples;
  data_samples.push_back(data_sample);
  return addTrainingData(data_samples, data_label, variable_names);
}

bool SVMClassifier::addTrainingData(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                                    const task_recorder2_msgs::DataSampleLabel& data_label,
                                    const std::vector<std::string> variable_names)
{
  std::vector<task_recorder2_msgs::DataSampleLabel> data_labels;
  for (int i = 0; i < (int)data_samples.size(); ++i)
  {
    data_labels.push_back(data_label);
  }
  return addTrainingData(data_samples, data_labels, variable_names);
}

bool SVMClassifier::addTrainingData(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                                    const std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels,
                                    const std::vector<std::string> variable_names)
{
  ROS_ASSERT_MSG(svm_parameters_.initialized_, "SVMClassifier is not initialized.");
  ROS_ASSERT_MSG(!data_samples.empty(), "No data samples provided.");
  ROS_ASSERT_MSG(data_samples.size() == data_labels.size(), "Number of samples must equal number of labels.");

  ROS_ASSERT_MSG(!trained_, "SVMClassifier is already trained, adding more data not supported yet.");
  ROS_ASSERT_MSG(!loaded_, "SVMClassifier is already loaded, adding more data not supported yet.");

  if (data_samples_.empty()) // the SVM was empty
  {
    reset();
    if (trained_)
    {
      freeSVM();
    }
    if (variable_names.empty()) // use all data
    {
      svm_parameters_.msg_.variable_names = data_samples.front().names;
    }
    else // only use provided variable names
    {
      svm_parameters_.msg_.variable_names = variable_names;
    }
  }
  else // there has been already data added
  {
    // error checking
    ROS_VERIFY(task_recorder2_utilities::haveSameContent(variable_names, svm_parameters_.msg_.variable_names));
  }

  ROS_VERIFY(task_recorder2_utilities::getIndices(data_samples.front().names, svm_parameters_.msg_.variable_names, svm_parameters_.msg_.indices));
  svm_parameters_.msg_.num_variables = (int)svm_parameters_.msg_.indices.size();
  for (int i = 0; i < (int)data_samples.size(); ++i)
  {
    task_recorder2_msgs::DataSample data_sample;
    for (int j = 0; j < svm_parameters_.msg_.num_variables; ++j)
    {
      data_sample.header = data_samples[i].header;
      data_sample.names.push_back(data_samples[i].names[svm_parameters_.msg_.indices[j]]);
      data_sample.data.push_back(data_samples[i].data[svm_parameters_.msg_.indices[j]]);
    }
    data_samples_.push_back(data_sample);
  }

  // add labels
  data_labels_.insert(data_labels_.end(), data_labels.begin(), data_labels.end());
  return true;
}

bool SVMClassifier::train()
{
  ROS_ASSERT_MSG(svm_parameters_.initialized_, "SVMClassifier is not initialized.");
  svm_parameters_.msg_.num_data_samples = (int)data_samples_.size();
  ROS_ASSERT_MSG(svm_parameters_.msg_.num_data_samples > 0, "No data samples added yet. Cannot train SVM.");
  ROS_ASSERT_MSG(svm_parameters_.msg_.num_data_samples == (int)data_labels_.size(), "Number of data samples >%i< must match number of data lables >%i<. Cannot train SVM.",
                 svm_parameters_.msg_.num_data_samples, (int)data_labels_.size());

  ROS_DEBUG("Training SVM with >%i< dimensions from >%i< samples.", svm_parameters_.msg_.num_variables, svm_parameters_.msg_.num_data_samples);

  // assuming that input data is somewhat normalized

//  if (SVM_LOGGING_ENABLED)
//  {
    std::vector<std::vector<double> > log_data;
    std::vector<double> log_label;
//  }

  // allocate memory and fill data
  training_labels_ = new float64_t[svm_parameters_.msg_.num_data_samples];
  training_features_ = new float64_t[svm_parameters_.msg_.num_data_samples * svm_parameters_.msg_.num_variables];
  for (int i = 0; i < svm_parameters_.msg_.num_data_samples; ++i)
  {
    double label;
    ROS_VERIFY(task_recorder2_utilities::getSVMLabel(data_labels_[i], label));
    training_labels_[i] = static_cast<float64_t> (label);
    if (SVM_LOGGING_ENABLED)
    {
      log_label.push_back(label);
      log_data.push_back(data_samples_[i].data);
    }
    for (int j = 0; j < svm_parameters_.msg_.num_variables; ++j)
    {
      training_features_[i * svm_parameters_.msg_.num_variables + j] = static_cast<float64_t> (data_samples_[i].data[j]);
    }
  }

  if(SVM_LOGGING_ENABLED)
  {
    usc_utilities::log(log_label, "/tmp/training_label.txt");
    usc_utilities::log(log_data, "/tmp/training_data.txt");
  }

  // create train labels
  ctraining_labels_ = new shogun::CLabels();
  shogun::SGVector<float64_t> labels(training_labels_, svm_parameters_.msg_.num_data_samples);
  ctraining_labels_->set_labels(labels);
  SG_REF(ctraining_labels_);
  ROS_ASSERT(ctraining_labels_->is_two_class_labeling());

  // create train features
  ctraining_features_ = new shogun::CSimpleFeatures<float64_t>(FEATURE_CACHE_SIZE);
  ctraining_features_->copy_feature_matrix(training_features_, svm_parameters_.msg_.num_variables, svm_parameters_.msg_.num_data_samples);
  SG_REF(ctraining_features_);

  // create gaussian kernel
  ROS_VERIFY(createKernel());
  ROS_VERIFY(ckernel_->init(ctraining_features_, ctraining_features_));

  // ckernel_->set_optimization_type(shogun::SLOWBUTMEMEFFICIENT);
  // ROS_ASSERT(ckernel_->get_is_initialized());

  // create svm
  ROS_VERIFY(createSVM());

  // train
  ROS_DEBUG("Training SVM.");
  try
  {
    svm_->train();
  }
  catch (shogun::ShogunException& ex)
  {
    ROS_ERROR("Could not train SVM : %s.", ex.get_exception_string());
    return (trained_ = false);
  }

  ROS_DEBUG("Training finished. There are >%d< support vectors and bias is >%f<.", svm_->get_num_support_vectors(), svm_->get_bias());

  // reset();
  return (trained_ = true);
}

bool SVMClassifier::createKernel()
{
  if(svm_parameters_.msg_.kernel_type == shogun::K_GAUSSIAN)
  {
    ckernel_ = new shogun::CGaussianKernel(KERNEL_CACHE_SIZE, svm_parameters_.msg_.kernel_width);
  }
//  else if(svm_parameters_.msg_.kernel_type == shogun::K_POWER)
//  {
//    ckernel_ = new shogun::CGaussianKernel(KERNEL_CACHE_SIZE, svm_parameters_.msg_.kernel_width);
//  }
  else if(svm_parameters_.msg_.kernel_type == shogun::K_LINEAR)
  {
    ckernel_ = new shogun::CLinearKernel();
  }
  else
  {
    ROS_ERROR("Unknown kernel type >%i<.", svm_parameters_.msg_.kernel_type);
    return false;
  }
  SG_REF(ckernel_);
  ROS_DEBUG("Created a kernel of type >%s<.", ckernel_->get_name());
  return true;
}

bool SVMClassifier::createSVM()
{
  ROS_ASSERT_MSG(svm_parameters_.initialized_, "SVM Parameters not initialized, cannor create SVM.");
  if(svm_parameters_.msg_.svm_lib == shogun::CT_LIBSVM)
  {
    svm_ = new shogun::CLibSVM(svm_parameters_.msg_.svm_c, ckernel_, ctraining_labels_);
  }
  else if(svm_parameters_.msg_.svm_lib == shogun::CT_LIBSVMONECLASS)
  {
    svm_ = new shogun::CLibSVMOneClass(svm_parameters_.msg_.svm_c, ckernel_);
  }
  else if(svm_parameters_.msg_.svm_lib == shogun::CT_LIGHT)
  {
    svm_ = new shogun::CSVMLight(svm_parameters_.msg_.svm_c, ckernel_, ctraining_labels_);
  }
  else if(svm_parameters_.msg_.svm_lib == shogun::CT_LIGHTONECLASS)
  {
    svm_ = new shogun::CSVMLightOneClass(svm_parameters_.msg_.svm_c, ckernel_);
  }
  else
  {
    ROS_ASSERT_MSG(false, "Unknown SVM lib identifier >%i<.", svm_parameters_.msg_.svm_lib);
  }
  ROS_DEBUG("Created SVM of type >%s<.", svm_->get_name());
  SG_REF(svm_);
  float64_t c_neg = 1.0;
  float64_t c_pos = 1.0;
  svm_->set_C(c_neg, c_pos);
  svm_->set_epsilon(svm_parameters_.msg_.svm_eps);
  return true;
}

bool SVMClassifier::save(const std::string directory_name)
{
  std::string dir = directory_name;
  usc_utilities::appendTrailingSlash(dir);
  ROS_DEBUG("Saving SVM to directory >%s<.", dir.c_str());

  // error checking
  if (!svm_parameters_.initialized_)
  {
    ROS_ERROR("SVM is not initialized. Cannot save it to file >%s<.", (dir + SVM_FILE_NAME).c_str());
    return false;
  }
  if (!trained_ && !loaded_)
  {
    ROS_ERROR("SVM is not trained and not loaded. Cannot save it to file >%s<.", (dir + SVM_FILE_NAME).c_str());
    return false;
  }

  // write SVM to disc
  ROS_DEBUG("Saving SVM parameters with >%i< dimension.", svm_parameters_.msg_.num_variables);
  ROS_VERIFY(svm_parameters_.save(dir));
  shogun::CSerializableAsciiFile ser_kernel_file(const_cast<char*> ((dir + KERNEL_FILE_NAME).c_str()), 'w');
  ckernel_->save_serializable(&ser_kernel_file);
  shogun::CAsciiFile label_file(const_cast<char*> ((dir + LABELS_FILE_NAME).c_str()), 'w');
  ctraining_labels_->save(&label_file);

  ROS_DEBUG("Writing SVM file to >%s<.", (dir + SVM_FILE_NAME).c_str());
  FILE *svm_out;
  svm_out = fopen((dir + SVM_FILE_NAME).c_str(), "w");
  if(!svm_out)
  {
    ROS_ERROR("Problems when opening file >%s<.", (dir + SVM_FILE_NAME).c_str());
    return false;
  }

  if (!svm_->save(svm_out))
  {
    ROS_ERROR("Problems when writing SVM to file >%s<.", (dir + SVM_FILE_NAME).c_str());
    fclose(svm_out);
    return false;
  }
  fclose(svm_out);

  ROS_DEBUG("Saved SVM to file >%s<.", (dir + SVM_FILE_NAME).c_str());
  return true;
}

bool SVMClassifier::load(const std::string directory_name)
{
  ROS_VERIFY(initialize());
  std::string dir = directory_name;
  usc_utilities::appendTrailingSlash(dir);

  ROS_VERIFY(svm_parameters_.load(dir));
  ROS_DEBUG("Loaded SVM parameters with >%i< dimension.", svm_parameters_.msg_.num_variables);

  ROS_VERIFY(createKernel());
  shogun::CSerializableAsciiFile ser_kernel_file(const_cast<char*> ((dir + KERNEL_FILE_NAME).c_str()), 'r');
  ckernel_->load_serializable(&ser_kernel_file);

//  ctraining_features_ = new shogun::CSimpleFeatures<float64_t>(FEATURE_CACHE_SIZE);
//  SG_REF(ctraining_features_);

  ctraining_labels_ = new shogun::CLabels();
  SG_REF(ctraining_labels_);
  shogun::CAsciiFile label_file(const_cast<char*> ((dir + LABELS_FILE_NAME).c_str()), 'r');
  ctraining_labels_->load(&label_file);

  // create svm
  ROS_VERIFY(createSVM());

  FILE *svm_in;
  svm_in = fopen((dir + SVM_FILE_NAME).c_str(), "r");
  if (!svm_->load(svm_in))
  {
    ROS_ERROR("Problems when reading SVM from file >%s<.", (dir + SVM_FILE_NAME).c_str());
    fclose(svm_in);
    return false;
  }
  fclose(svm_in);

  ROS_DEBUG("Loaded SVM from file >%s<.", (dir + SVM_FILE_NAME).c_str());
  return (loaded_ = true);
}

bool SVMClassifier::predict(const task_recorder2_msgs::DataSample& data_sample,
                            task_recorder2_msgs::DataSampleLabel& data_label)/*,
                            const std::vector<std::string> variable_names)*/
{
  std::vector<task_recorder2_msgs::DataSample> data_samples;
  data_samples.push_back(data_sample);
  std::vector<task_recorder2_msgs::DataSampleLabel> data_labels;
  data_labels.push_back(data_label);
  ROS_VERIFY(predict(data_samples, data_labels/*, variable_names*/));
  ROS_ASSERT(data_labels.size() == 1);
  data_label = data_labels[0];
  return true;
}

bool SVMClassifier::predict(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                            std::vector<task_recorder2_msgs::DataSampleLabel>& data_labels)/*,
                            const std::vector<std::string> variable_names)*/
{

  // error checking
  ROS_ASSERT_MSG(trained_ || loaded_, "SVMClassifier is not trained or loaded.");
  ROS_ASSERT_MSG(!data_samples.empty(), "Data samples are empty, cannot predict anything.");
  // ROS_ASSERT_MSG(!variable_names.empty(), "No variable names provided.");

  std::vector<task_recorder2_msgs::DataSample> reduced_data_samples;
//  if (!variable_names.empty())
//  {
    ROS_VERIFY(task_recorder2_utilities::getIndices(data_samples[0].names, svm_parameters_.msg_.variable_names, svm_parameters_.msg_.indices));
    ROS_ASSERT_MSG(svm_parameters_.msg_.num_variables == (int)svm_parameters_.msg_.indices.size(),
                   "Number of variables used when training the SVM >%i< does not correspond to number of variables provided >%i<",
                   svm_parameters_.msg_.num_variables, (int)svm_parameters_.msg_.indices.size());

//    ROS_ASSERT_MSG(svm_parameters_.msg_.variable_names.size() == svm_parameters_.msg_.indices.size(),
//                   "Number of variables names provided is >%i<, but SVM has been trained for >%i< dimensions.",
//                   (int)variable_names.size(), (int)svm_parameters_.msg_.indices.size());
    for (int i = 0; i < (int)data_samples.size(); ++i)
    {
      task_recorder2_msgs::DataSample data_sample;
      for (int j = 0; j < svm_parameters_.msg_.num_variables; ++j)
      {
        data_sample.header = data_samples[i].header;
        data_sample.names.push_back(data_samples[i].names[svm_parameters_.msg_.indices[j]]);
        data_sample.data.push_back(data_samples[i].data[svm_parameters_.msg_.indices[j]]);
      }
      reduced_data_samples.push_back(data_sample);
    }
//  }

  int num_test_data_samples = 0;
  //  if (variable_names.empty())
  //  {
  //    num_test_data_samples = (int)data_samples.size();
  //    if ((int)data_samples[0].data.size() != NUM_VARS_DURING_TRAINING)
  //    {
  //      ROS_ERROR("The SVM has been trained for >%i< dimensions. Cannot predict for >%i< dimensions.",
  //                NUM_VARS_DURING_TRAINING, (int)data_samples[0].data.size());
  //      return false;
  //    }
  //  }
  //  else
  //  {
  num_test_data_samples = (int)reduced_data_samples.size();
  if ((int)reduced_data_samples[0].data.size() != svm_parameters_.msg_.num_variables)
  {
    ROS_ERROR("The SVM has been trained for >%i< dimensions. Cannot predict for >%i< dimensions.",
              svm_parameters_.msg_.num_variables, (int)reduced_data_samples[0].data.size());
    return false;
  }
  //  }

  int num_unclassiefied_data_samples = num_test_data_samples;
  int index = 0;
  int counter = 0;

//  if (SVM_LOGGING_ENABLED)
//  {
    std::vector<std::vector<double> > test_data_samples;
    std::vector<double> test_data_sample;
    test_data_sample.resize(svm_parameters_.msg_.num_variables);
    std::vector<double> predicted_labels;
    predicted_labels.resize(num_test_data_samples);
    data_labels.resize(num_test_data_samples);
//  }

  if ((int)data_labels.size() != num_test_data_samples)
  {
    data_labels.resize(num_test_data_samples);
  }

  for (int i = 0; i < num_test_data_samples; ++i)
  {
    if (index >= MAX_NUM_TEST_SAMPLES)
    {
      index = 0;
      // predict
      ctest_features_->copy_feature_matrix(test_features_, svm_parameters_.msg_.num_variables, MAX_NUM_TEST_SAMPLES);
      ctest_labels_ = svm_->apply(ctest_features_);
      // ROS_ASSERT(ctest_labels_->is_two_class_labeling());

      for (int j = 0; j < MAX_NUM_TEST_SAMPLES; ++j)
      {
        ROS_ASSERT_MSG(j < ctest_labels_->get_num_labels(), "You asked for label >%i<, but there are only >%i< labels.", j, ctest_labels_->get_num_labels());
        ROS_VERIFY(getLabel(ctest_labels_->get_label(j), data_labels[j + (counter * MAX_NUM_TEST_SAMPLES)]));

        if(SVM_LOGGING_ENABLED)
        {
          predicted_labels[j + counter * MAX_NUM_TEST_SAMPLES] = data_labels[j + counter * MAX_NUM_TEST_SAMPLES].binary_label.label;
        }
      }
      num_unclassiefied_data_samples -= MAX_NUM_TEST_SAMPLES;
      counter++;
    }

    for (int j = 0; j < svm_parameters_.msg_.num_variables; ++j)
    {
      if (svm_parameters_.msg_.variable_names.empty())
      {
        test_features_[index * svm_parameters_.msg_.num_variables + j] = static_cast<float64_t> (data_samples[i].data[j]);
      }
      else
      {
        test_features_[index * svm_parameters_.msg_.num_variables + j] = static_cast<float64_t> (reduced_data_samples[i].data[j]);
      }
      if(SVM_LOGGING_ENABLED)
      {
        test_data_sample[j] = static_cast<float64_t> (test_features_[index * svm_parameters_.msg_.num_variables + j]);
      }
    }
    if(SVM_LOGGING_ENABLED)
    {
      test_data_samples.push_back(test_data_sample);
    }
    index++;
  }

  if (num_unclassiefied_data_samples > 0)
  {
    // predict
    ctest_features_->copy_feature_matrix(test_features_, svm_parameters_.msg_.num_variables, num_unclassiefied_data_samples);
    ctest_labels_ = svm_->apply(ctest_features_);
    // ROS_ASSERT(ctest_labels_->is_two_class_labeling());

    for (int i = 0; i < num_unclassiefied_data_samples; ++i)
    {
      ROS_ASSERT_MSG(i < ctest_labels_->get_num_labels(), "You asked for label >%i<, but there are only >%i< labels.", i, ctest_labels_->get_num_labels());
      ROS_VERIFY(getLabel(ctest_labels_->get_label(i), data_labels[i + (counter * MAX_NUM_TEST_SAMPLES)]));

      if(SVM_LOGGING_ENABLED)
      {
        predicted_labels[i + counter * MAX_NUM_TEST_SAMPLES] = data_labels[i + counter * MAX_NUM_TEST_SAMPLES].binary_label.label;
      }
    }
  }

  if(SVM_LOGGING_ENABLED)
  {
    usc_utilities::log(predicted_labels, "/tmp/predicted_labels.txt");
    usc_utilities::log(test_data_samples, "/tmp/test_data.txt");
  }
  return true;
}

bool SVMClassifier::setClassificationBoundary(const double classification_boundary)
{
  ROS_ASSERT(svm_parameters_.initialized_);
  svm_parameters_.msg_.classification_boundary = classification_boundary;
  return true;
}

double SVMClassifier::getClassificationBoundary() const
{
  ROS_ASSERT(svm_parameters_.initialized_);
  return svm_parameters_.msg_.classification_boundary;
}

bool SVMClassifier::getLabel(const double value,
                             task_recorder2_msgs::DataSampleLabel& label)
{
  return task_recorder2_utilities::getBinaryLabel(value, label, svm_parameters_.msg_.classification_boundary);
}

bool SVMClassifier::setSVMParametersMsg(const SVMParametersMsg& msg)
{
  svm_parameters_.msg_ = msg;
  return true;
}

bool SVMClassifier::getSVMParametersMsg(SVMParametersMsg& msg) const
{
  if(!svm_parameters_.initialized_)
  {
    return false;
  }
  msg = svm_parameters_.msg_;
  return true;
}

}
