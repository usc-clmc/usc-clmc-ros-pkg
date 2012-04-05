/*!
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Heiko Strathmann
 * Copyright (C) 2011 Berlin Institute of Technology and Max-Planck-Society
 */
/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   modelselection_grid_search_kernel.cpp

  \author Peter Pastor
  \date   Jul 22, 2011

 *********************************************************************/

// system includes
#include <sstream>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/logging.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_utilities/data_sample_utilities.h>

// local includes
#include <task_event_detector/modelselection_grid_search_kernel.h>
#include <task_event_detector/svm_trainer.h>
#include <task_event_detector/data_sample_filter.h>

// SVMLight has stupid global define causing trouble with VERSION string in rosbag.h
#include <shogun/classifier/svm/LibSVM.h>
#include <shogun/classifier/svm/LibSVMOneClass.h>
#include <shogun/classifier/svm/SVMLight.h>
#include <shogun/classifier/svm/SVMLightOneClass.h>

using namespace shogun;

namespace task_event_detector
{

ModelSelectionGridSearchKernel::ModelSelectionGridSearchKernel(ros::NodeHandle node_handle) :
  node_handle_(node_handle), initialized_(false), monitor_io_(ros::NodeHandle("/TaskRecorderManager"))
{
  ROS_VERIFY(monitor_io_.initialize());
  ROS_INFO("Running model selection in namespace >%s<.", node_handle_.getNamespace().c_str());
}

bool ModelSelectionGridSearchKernel::readParameters(SVMParametersMsg& msg)
{
  int type;
  ROS_VERIFY(usc_utilities::read(node_handle_, "classifier_type", type));
  if(type == CT_LIGHT)
  {
    msg.svm_lib = CT_LIGHT;
  }
  else if(type == CT_LIBSVM)
  {
    msg.svm_lib = CT_LIBSVM;
  }
  else
  {
    ROS_ERROR("Unknown classifier type >%i<.", type);
    return false;
  }

  bool use_gaussian_kernel;
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_gaussian_kernel", use_gaussian_kernel));
  msg.grid_search_parameters.use_gaussian_kernel = use_gaussian_kernel;
  bool use_linear_kernel;
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_linear_kernel", use_linear_kernel));
  msg.grid_search_parameters.use_linear_kernel = use_linear_kernel;

  ROS_VERIFY(usc_utilities::read(node_handle_, "num_subsets", msg.grid_search_parameters.num_subsets));
  ROS_VERIFY(usc_utilities::read(node_handle_, "detection_variable_names", msg.variable_names));
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_runs", msg.grid_search_parameters.num_runs));
  ROS_VERIFY(usc_utilities::read(node_handle_, "conf_int_alpha", msg.grid_search_parameters.conf_int_alpha));

  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_C_min", msg.grid_search_parameters.cv_svm_C_min));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_C_max", msg.grid_search_parameters.cv_svm_C_max));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_C_range_type", msg.grid_search_parameters.cv_svm_C_range_type));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_C_base", msg.grid_search_parameters.cv_svm_C_base));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_C_step_size", msg.grid_search_parameters.cv_svm_C_step_size));

  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_min", msg.grid_search_parameters.cv_svm_width_min));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_max", msg.grid_search_parameters.cv_svm_width_max));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_range_type", msg.grid_search_parameters.cv_svm_width_range_type));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_base", msg.grid_search_parameters.cv_svm_width_base));
  ROS_VERIFY(usc_utilities::read(node_handle_, "cv_svm_width_step_size", msg.grid_search_parameters.cv_svm_width_step_size));

  return true;
}

CModelSelectionParameters* ModelSelectionGridSearchKernel::createParamTree(const SVMParametersMsg& msg)
{

  CModelSelectionParameters* root = new CModelSelectionParameters();

  CModelSelectionParameters* c1 = new CModelSelectionParameters("C1");
  // This is so ugly, this should be hidden.

  shogun::ERangeType svm_c_range_type;
  ROS_VERIFY(getRangeType(msg.grid_search_parameters.cv_svm_C_range_type, svm_c_range_type));
  c1->build_values(msg.grid_search_parameters.cv_svm_C_min, msg.grid_search_parameters.cv_svm_C_max,
                   svm_c_range_type, msg.grid_search_parameters.cv_svm_C_step_size, msg.grid_search_parameters.cv_svm_C_base);
  root->append_child(c1);

  // Linear kernel
  if(msg.grid_search_parameters.use_linear_kernel)
  {
    CLinearKernel* linear_kernel = new CLinearKernel();
    CModelSelectionParameters* param_linear_kernel = new CModelSelectionParameters("kernel", linear_kernel);
    root->append_child(param_linear_kernel);
  }

  // Gaussian kernel
  if(msg.grid_search_parameters.use_gaussian_kernel)
  {
    CGaussianKernel* gaussian_kernel = new CGaussianKernel();
    CModelSelectionParameters* param_gaussian_kernel = new CModelSelectionParameters("kernel", gaussian_kernel);
    CModelSelectionParameters* gaussian_kernel_width = new CModelSelectionParameters("width");
    shogun::ERangeType svm_width_range_type;
    ROS_VERIFY(getRangeType(msg.grid_search_parameters.cv_svm_width_range_type, svm_width_range_type));
    gaussian_kernel_width->build_values(msg.grid_search_parameters.cv_svm_width_min, msg.grid_search_parameters.cv_svm_width_max,
                                        svm_width_range_type, msg.grid_search_parameters.cv_svm_width_step_size,
                                        msg.grid_search_parameters.cv_svm_width_base);
    param_gaussian_kernel->append_child(gaussian_kernel_width);
    root->append_child(param_gaussian_kernel);
  }

  return root;
}

bool ModelSelectionGridSearchKernel::readData(const std::vector<task_recorder2_msgs::Description>& data_descriptions,
                                              const std::vector<std::string>& detection_variable_names)
{

  data_samples_.clear();
  data_sample_labels_.clear();

  // read all data
  std::vector<task_recorder2_msgs::DataSample> all_data_samples;
  for (int i = 0; i < (int)data_descriptions.size(); ++i)
  {
    task_recorder2_msgs::DataSampleLabel data_sample_label;
    std::vector<task_recorder2_msgs::DataSample> data_samples;
    ROS_VERIFY(monitor_io_.readData(data_descriptions[i], data_samples, data_sample_label));
    ROS_ASSERT_MSG(!data_samples.empty(), "No data samples read from disc.");
    all_data_samples.insert(all_data_samples.end(), data_samples.begin(), data_samples.end());
    for (int i = 0; i < (int)data_samples.size(); ++i)
    {
      data_sample_labels_.push_back(data_sample_label);
    }
  }
  ROS_ASSERT_MSG(!all_data_samples.empty(), "No data samples read.");
  ROS_VERIFY(applyFilter(all_data_samples));

  ROS_VERIFY(task_recorder2_utilities::extractDataSamples(all_data_samples, detection_variable_names, data_samples_));
  ROS_ASSERT(data_samples_.size() == data_sample_labels_.size());
  ROS_DEBUG("Read total of >%i< labeled data samples read from files...", (int)data_samples_.size());
  return true;
}

bool ModelSelectionGridSearchKernel::applyFilter(std::vector<task_recorder2_msgs::DataSample>& data_samples)
{
  ROS_ASSERT_MSG(!data_samples.empty(), "No data samples provided.");
  DataSampleFilter data_sample_filter;
  ROS_VERIFY(data_sample_filter.initialize(data_samples.front()));
  for (int i = 0; i < (int)data_samples.size(); ++i)
  {
    ROS_VERIFY(data_sample_filter.filter(data_samples[i]));
  }
  return true;
}

bool ModelSelectionGridSearchKernel::crossvalidate(const task_recorder2_msgs::Description& svm_description,
                                                   const std::vector<task_recorder2_msgs::Description>& data_descriptions,
                                                   SVMParametersMsg& parameters,
                                                   std::string& info)
{
  description_ = svm_description;

  // error checking
  if(parameters.variable_names.empty())
  {
    info.assign("No variable names provided.");
    ROS_INFO_STREAM(info);
    return false;
  }
  if(!parameters.grid_search_parameters.use_linear_kernel && !parameters.grid_search_parameters.use_gaussian_kernel)
  {
    info.assign("Either linear or gaussian kernel must be used.");
    return false;
  }

  if(!readData(data_descriptions, parameters.variable_names))
  {
    info.assign("Problems when reading data.");
    ROS_INFO_STREAM(info);
    return false;
  }

  const int NUM_DATA_SAMPLES = (int)data_samples_.size();
  const int NUM_VARIABLES = (int)data_samples_[0].data.size();
  ROS_INFO("Setting up model selection for >%i< data samples with >%i< features each.", NUM_DATA_SAMPLES, NUM_VARIABLES);

  float64_t* matrix = new float64_t[NUM_DATA_SAMPLES * NUM_VARIABLES];
  for (int i = 0; i < NUM_DATA_SAMPLES; ++i)
  {
    for (int j = 0; j < NUM_VARIABLES; ++j)
    {
      matrix[i * NUM_VARIABLES + j] = data_samples_[i].data[j];
    }
  }
  CSimpleFeatures<float64_t>* features = new CSimpleFeatures<float64_t> ();
  features->set_feature_matrix(matrix, NUM_VARIABLES, NUM_DATA_SAMPLES);

  float64_t* training_labels = new float64_t[NUM_DATA_SAMPLES];
  for (int i = 0; i < NUM_DATA_SAMPLES; ++i)
  {
    ROS_ASSERT_MSG(data_sample_labels_[i].type == task_recorder2_msgs::DataSampleLabel::BINARY_LABEL, "Data sample label must be of type BINARY_LABEL.");
    training_labels[i] = (float64_t)data_sample_labels_[i].binary_label.label;
    ROS_DEBUG("Set label >%i< to >%i<.", i, data_sample_labels_[i].binary_label.label);
  }

  shogun::SGVector<float64_t> label_vector(training_labels, NUM_DATA_SAMPLES);
  CLabels* labels = new CLabels(NUM_DATA_SAMPLES);
  labels->set_labels(label_vector);
  for (int i = 0; i < NUM_DATA_SAMPLES; ++i)
  {
    ROS_DEBUG("Set label >%i< to >%f<.", i, labels->get_label(i));
  }
  ROS_VERIFY_MSG(labels->is_two_class_labeling(), "Labels only contain labels of 1 class.");

  CSVM* classifier;
  if(parameters.svm_lib == CT_LIBSVM)
  {
    classifier = new CLibSVM();
  }
  else if(parameters.svm_lib == CT_LIGHT)
  {
    classifier = new CSVMLight();
  }
  else
  {
    ROS_ERROR("Unknown classifier type >%i<. This should never happen.", (int)parameters.svm_lib);
    return false;
  }

  // splitting strategy
  CStratifiedCrossValidationSplitting* splitting_strategy = new CStratifiedCrossValidationSplitting(labels, parameters.grid_search_parameters.num_subsets);

  // accuracy evaluation
  CContingencyTableEvaluation* evaluation_criterium = new CContingencyTableEvaluation(ACCURACY);
  // ROC evaluation
  // CROCEvaluation* evaluation_criterium = new CROCEvaluation();

  // cross validation class for evaluation in model selection
  CCrossValidation* cross = new CCrossValidation(classifier, features, labels, splitting_strategy, evaluation_criterium);

  CMachine* machine = cross->get_machine();
  ROS_INFO("Using >%s< classifier...", machine->get_name());

  // model parameter selection, deletion is handled by modsel class (SG_UNREF)
  CModelSelectionParameters* param_tree = createParamTree(parameters);
  param_tree->print_tree();

  // this is on the stack and handles all of the above structures in memory
  CGridSearchModelSelection grid_search(param_tree, cross);

  //  shogun::Parallel* parallel = new Parallel() ;
  //  SG_REF(parallel);
  //  parallel->set_num_threads(parallel->get_num_cpus());
  //  grid_search.set_global_parallel(parallel);
  //  SG_SPRINT("Starting grid search...\n");

  CParameterCombination* best_combination = grid_search.select_model();
  SG_SPRINT("Best parameter(s):\n");
  best_combination->print_tree();
  best_combination->apply_to_machine(classifier);

  // larger number of runs to have tighter confidence intervals
  cross->set_num_runs(parameters.grid_search_parameters.num_runs);
  cross->set_conf_int_alpha(parameters.grid_search_parameters.conf_int_alpha);

  // get result
  SG_SPRINT("Evaluating result:\n");
  CrossValidationResult result = cross->evaluate();
  SG_SPRINT("Result: ");
  result.print_result();

  // assign best parameters
  parameters.kernel_type = (int)classifier->get_kernel()->get_kernel_type();
  parameters.svm_c = classifier->get_C1();
  parameters.svm_eps = classifier->get_epsilon();
  if (classifier->get_kernel()->get_kernel_type() == K_LINEAR)
  {

  }
  else if (classifier->get_kernel()->get_kernel_type() == K_GAUSSIAN)
  {
    CGaussianKernel* gk = dynamic_cast<CGaussianKernel*>(classifier->get_kernel());
    parameters.kernel_width = gk->get_width();
  }

  // assign info
  std::string nl = "<br />";

  info.assign("==============================================" + nl);
  info.append("Result of cross validation:" + nl);
  info.append("Accuracy: " + getString(evaluation_criterium->get_accuracy()) + nl);
  info.append("Error rate: " + getString(evaluation_criterium->get_error_rate()) + nl);
  info.append("Precision: " + getString(evaluation_criterium->get_precision()) + nl);
  info.append("Specificity: " + getString(evaluation_criterium->get_specificity()) + nl);
  info.append("==============================================" + nl);
  info.append("Bias: " + getString(classifier->get_bias()) + nl);
  info.append("Epsilon: " + getString(classifier->get_epsilon()) + nl);
  info.append("Number of support vectors: " + getString(classifier->get_num_support_vectors()) + nl);
  info.append("Epsilon: " + getString(parameters.svm_eps) + nl);
  info.append("Kernel class: " + std::string(classifier->get_kernel()->get_name()) + nl);
  info.append("Kernel width: " + getString(parameters.kernel_width) + nl);
  info.append("Regularization C: " + getString(parameters.svm_c) + nl);
  info.append("==============================================" + nl);

  ROS_INFO("===================================================================");
  ROS_INFO("Accuracy: %f", evaluation_criterium->get_accuracy());
  ROS_INFO("Error rate: %f", evaluation_criterium->get_error_rate());
  ROS_INFO("Precision: %f", evaluation_criterium->get_precision());
  ROS_INFO("Specificity: %f", evaluation_criterium->get_specificity());
  ROS_INFO("Cross correlation: %f", evaluation_criterium->get_cross_correlation());
  ROS_INFO("WRACC: %f", evaluation_criterium->get_WRACC());
  ROS_INFO("F1: %f", evaluation_criterium->get_F1());
  ROS_INFO("BAL: %f", evaluation_criterium->get_BAL());
  SGVector<char*> names = evaluation_criterium->get_modelsel_names();
  for (int i = 0; i < (int)names.vlen; ++i)
  {
    ROS_INFO("Name(%i): %s.", i, names.vector[i]);
  }
  double C1 = classifier->get_C1();
  ROS_INFO("C1: %f", C1);
  ROS_INFO("Kernel class is >%s<.", classifier->get_kernel()->get_name());
  ROS_INFO("===================================================================");

  //  SGMatrix<double> roc = evaluation_criterium->get_ROC();
  //  ROS_INFO("Got ROC matrix with >%i< rows and >%i< cols.", roc.num_rows, roc.num_cols);
  //  ROS_INFO("Area under the ROC curve is >%f<.", evaluation_criterium->get_auROC());
  //  ROS_VERIFY(roc.num_rows == 2);
  //  std::vector<std::vector<double> > roc_matrix;
  //  for (int j = 0; j < roc.num_cols; ++j)
  //  {
  //    std::vector<double> roc_row;
  //    roc_row.push_back(roc.matrix[2*j]);
  //    roc_row.push_back(roc.matrix[2*j+1]);
  //    roc_matrix.push_back(roc_row);
  //  }
  //  usc_utilities::log(roc_matrix, "/tmp/roc.txt");

  SVMTrainer svm_trainer(description_, parameters);
  if(!svm_trainer.train(data_samples_, data_sample_labels_))
  {
    ROS_ERROR("Could not train SVM.");
    return false;
  }

  // clean up destroy result parameter
  // SG_UNREF(parallel);
  SG_UNREF(best_combination);
  return true;
}

std::vector<double> ModelSelectionGridSearchKernel::getLine(const double min_exp,
                                                            const double max_exp,
                                                            const shogun::ERangeType range_type,
                                                            const int num_steps,
                                                            const double base)
{
  std::vector<double> line;
  ROS_ASSERT(min_exp < max_exp);
  ROS_ASSERT(num_steps >= 2);

  if(range_type == shogun::R_EXP)
  {
    double lin_diff = (max_exp - min_exp) / (num_steps-1);
    for (int i = 0; i < num_steps; ++i)
    {
      double exponent = min_exp + i * lin_diff;
      line.push_back( pow(base, exponent) );
    }
  }
  else if(range_type == shogun::R_LINEAR)
  {
    double lin_diff = (max_exp - min_exp) / (num_steps-1);
    for (int i = 0; i < num_steps; ++i)
    {
      line.push_back(min_exp + i * lin_diff);
    }
  }
  else
  {
    ROS_ASSERT_MSG(false, "Range type >%i< not implemented yet.", (int)range_type);
  }

  return line;
}

std::string ModelSelectionGridSearchKernel::getString(const int number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}
std::string ModelSelectionGridSearchKernel::getString(const double number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

bool ModelSelectionGridSearchKernel::getRangeType(const int range_type_int, shogun::ERangeType& range_type)
{
  if(range_type_int == R_LINEAR)
  {
    range_type = R_LINEAR;
  }
  else if(range_type_int == R_EXP)
  {
    range_type = R_EXP;
  }
  else if(range_type_int == R_LOG)
  {
    range_type = R_LOG;
  }
  else
  {
    ROS_ERROR("Unknown range type >%i<.", range_type_int);
    return false;
  }
  return true;
}

bool ModelSelectionGridSearchKernel::getRangeType(const std::string& range_type_string, shogun::ERangeType& range_type)
{
  if(range_type_string.compare("lin") == 0)
  {
    range_type = R_LINEAR;
  }
  else if(range_type_string.compare("exp") == 0)
  {
    range_type = R_EXP;
  }
  else if(range_type_string.compare("log") == 0)
  {
    range_type = R_LOG;
  }
  else
  {
    ROS_ERROR("Unknown range type >%s<. (It should be either of 'lin', 'exp', and 'log').", range_type_string.c_str());
    return false;
  }
  return true;
}

}

