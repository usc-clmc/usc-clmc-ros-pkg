/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks Check:
           http://www.csie.ntu.edu.tw/~cjlin/papers/guide/guide.pdf
           http://www.csie.ntu.edu.tw/~cjlin/papers/libsvm.pdf

  \file   modelselection_grid_search_kernel.h

  \author Peter Pastor
  \date   Jul 22, 2011

 *********************************************************************/

#ifndef MODELSELECTION_GRID_SEARCH_KERNEL_H_
#define MODELSELECTION_GRID_SEARCH_KERNEL_H_

// system includes
#include <ros/ros.h>

#include <shogun/evaluation/CrossValidation.h>
#include <shogun/evaluation/ContingencyTableEvaluation.h>
#include <shogun/evaluation/StratifiedCrossValidationSplitting.h>

#include <shogun/modelselection/GridSearchModelSelection.h>
#include <shogun/modelselection/ModelSelectionParameters.h>
#include <shogun/modelselection/ParameterCombination.h>
#include <shogun/features/Labels.h>
#include <shogun/features/SimpleFeatures.h>

#include <shogun/kernel/LinearKernel.h>
#include <shogun/kernel/GaussianKernel.h>
#include <shogun/kernel/PowerKernel.h>
#include <shogun/distance/MinkowskiMetric.h>

#include <task_recorder2_utilities/task_monitor_io.h>
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_msgs/Description.h>

// local includes
#include <task_event_detector/SVMParametersMsg.h>
#include <task_event_detector/GridSearchParametersMsg.h>

namespace task_event_detector
{

class ModelSelectionGridSearchKernel
{

public:

  /*! Constructor
   */
  ModelSelectionGridSearchKernel(ros::NodeHandle node_handle = ros::NodeHandle("/TaskEventDetector"));
  /*! Destructor
   */
  virtual ~ModelSelectionGridSearchKernel() {};

  /*!
   * @param parameters
   * @return True on success, otherwise False
   */
  bool readParameters(SVMParametersMsg& parameters);

  /*!
   * @param parameters
   * @return True on success, otherwise False
   */
  bool getSVMParametersMsg(SVMParametersMsg& parameters) const;

  /*!
   * @param parameters
   * @return True on success, otherwise False
   */
  bool setSVMParametersMsg(const SVMParametersMsg& parameters);

  /*!
   * @param svm_description
   * @param data_descriptions
   * @param parameters
   * @param info
   * @return True on success, otherwise False
   */
  bool crossvalidate(const task_recorder2_msgs::Description& svm_description,
                     const std::vector<task_recorder2_msgs::Description>& data_descriptions,
                     SVMParametersMsg& parameters,
                     std::string& info);

private:

  ros::NodeHandle node_handle_;
  bool initialized_;
  shogun::CModelSelectionParameters* createParamTree(const SVMParametersMsg& msg);
  task_recorder2_utilities::TaskMonitorIO<task_recorder2_msgs::DataSample, task_recorder2_msgs::DataSampleLabel> monitor_io_;

  bool readData(const std::vector<task_recorder2_msgs::Description>& data_descriptions,
                const std::vector<std::string>& detection_variable_names);
  bool readParams(ros::NodeHandle node_handle);
  bool applyFilter(std::vector<task_recorder2_msgs::DataSample>& data_samples);
  bool getRangeType(const std::string& range_type_string, shogun::ERangeType& range_type);
  bool getRangeType(const int range_type_int, shogun::ERangeType& range_type);

  task_recorder2_msgs::Description description_;
  std::vector<task_recorder2_msgs::DataSample> data_samples_;
  std::vector<task_recorder2_msgs::DataSampleLabel> data_sample_labels_;

  SVMParametersMsg parameters_;

  std::vector<double> getLine(const double min,
                              const double max,
                              const shogun::ERangeType range_type,
                              const int num_steps,
                              const double base);

  std::string getString(const int number);
  std::string getString(const double number);

};

}

#endif /* MODELSELECTION_GRID_SEARCH_KERNEL_H_ */
