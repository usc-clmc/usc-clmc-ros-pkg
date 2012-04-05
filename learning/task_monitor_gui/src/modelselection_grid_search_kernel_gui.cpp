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
#include <boost/shared_ptr.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/logging.h>

#include <task_recorder2_utilities/data_sample_utilities.h>

#include <task_event_detector/SVMParametersMsg.h>
// #include <task_event_detector/svm_trainer.h>

// local includes
#include <task_monitor_gui/modelselection_grid_search_kernel_gui.h>

namespace task_monitor_gui
{

ModelSelectionGridSearchKernelGui::ModelSelectionGridSearchKernelGui(ros::NodeHandle node_handle,
                                                                     QWidget* parent,
                                                                     Qt::WFlags flags) :
  QMainWindow(parent, flags), node_handle_(node_handle)
{

  // this sets up GUI
  setupUi(this);

  // signals/slots mechanism in action
  ROS_VERIFY(connect(load_default_button_, SIGNAL(clicked()), this, SLOT(loadDefault())));
  ROS_VERIFY(connect(done_button_, SIGNAL(clicked()), this, SLOT(done())));
  ROS_VERIFY(connect(crossvalidate_button_, SIGNAL(clicked()), this, SLOT(crossvalidate())));

}

void ModelSelectionGridSearchKernelGui::loadDefault()
{
  task_event_detector::SVMParametersMsg msg;
  if(!model_selection_grid_search_kernel_.readParameters(msg))
  {
    setStatusReport("Could not read default parameters.", ERROR);
  }
  if(!setSVMParametersMsg(msg))
  {
    setStatusReport("Problems when setting parameters.", ERROR);
  }
}

void ModelSelectionGridSearchKernelGui::crossvalidate()
{
  clearStatusReport();
  if(!getSVMParametersMsg(parameters_))
  {
    setStatusReport("Could not get SVM parameters.", ERROR);
    return;
  }

  std::string info;
  if(!model_selection_grid_search_kernel_.crossvalidate(svm_description_, data_descriptions_, parameters_, info))
  {
    setStatusReport(info, ERROR);
  }
  else
  {
    setSVMParametersMsg(parameters_);
    setStatusReport(info, INFO);
  }
}

bool ModelSelectionGridSearchKernelGui::setup(const task_recorder2_msgs::Description& svm_description,
                                              const std::vector<task_recorder2_msgs::Description>& data_descriptions,
                                              task_event_detector::SVMParametersMsg& parameters,
                                              QObject* qobject)
{
  connect(this, SIGNAL(signalDone(const task_recorder2_msgs::Description&, const task_event_detector::SVMParametersMsg&)),
          qobject, SLOT(doneCrossvalidating(const task_recorder2_msgs::Description&, const task_event_detector::SVMParametersMsg&)));
  svm_description_ = svm_description;
  data_descriptions_ = data_descriptions;
  parameters_ = parameters;
  setSVMParametersMsg(parameters_);
  clearStatusReport();
  show();
  return true;
}

void ModelSelectionGridSearchKernelGui::done()
{
  hide();
  getSVMParametersMsg(parameters_);
  signalDone(svm_description_, parameters_);
}

bool ModelSelectionGridSearchKernelGui::getSVMParametersMsg(task_event_detector::SVMParametersMsg& msg)
{
  msg.grid_search_parameters.use_linear_kernel = use_linear_kernel_checkbox_->isChecked();
  msg.grid_search_parameters.use_gaussian_kernel = use_gaussian_kernel_checkbox_->isChecked();

  msg.grid_search_parameters.num_subsets = num_subsets_box_->value();
  msg.grid_search_parameters.num_runs = num_runs_box_->value();
  msg.grid_search_parameters.conf_int_alpha = conf_int_alpha_box_->value();

  msg.grid_search_parameters.cv_svm_C_min = c_min_box_->value();
  msg.grid_search_parameters.cv_svm_C_max = c_max_box_->value();
  getRange(c_range_type_box_, msg.grid_search_parameters.cv_svm_C_range_type);
  msg.grid_search_parameters.cv_svm_C_step_size = c_step_size_box_->value();
  msg.grid_search_parameters.cv_svm_C_base = c_base_box_->value();

  msg.grid_search_parameters.cv_svm_width_min = width_min_box_->value();
  msg.grid_search_parameters.cv_svm_width_max = width_max_box_->value();
  getRange(width_range_type_box_, msg.grid_search_parameters.cv_svm_width_range_type);
  msg.grid_search_parameters.cv_svm_width_step_size = width_step_size_box_->value();
  msg.grid_search_parameters.cv_svm_width_base = width_base_box_->value();
  return true;
}

bool ModelSelectionGridSearchKernelGui::setSVMParametersMsg(const task_event_detector::SVMParametersMsg& msg)
{
  use_linear_kernel_checkbox_->setChecked(msg.grid_search_parameters.use_linear_kernel);
  use_gaussian_kernel_checkbox_->setChecked(msg.grid_search_parameters.use_gaussian_kernel);

  num_subsets_box_->setValue(msg.grid_search_parameters.num_subsets);
  num_runs_box_->setValue(msg.grid_search_parameters.num_runs);
  conf_int_alpha_box_->setValue(msg.grid_search_parameters.conf_int_alpha);

  c_min_box_->setValue(msg.grid_search_parameters.cv_svm_C_min);
  c_max_box_->setValue(msg.grid_search_parameters.cv_svm_C_max);
  setRange(c_range_type_box_, msg.grid_search_parameters.cv_svm_C_range_type);
  c_step_size_box_->setValue(msg.grid_search_parameters.cv_svm_C_step_size);
  c_base_box_->setValue(msg.grid_search_parameters.cv_svm_C_base);

  width_min_box_->setValue(msg.grid_search_parameters.cv_svm_width_min);
  width_max_box_->setValue(msg.grid_search_parameters.cv_svm_width_max);
  setRange(width_range_type_box_, msg.grid_search_parameters.cv_svm_width_range_type);
  width_step_size_box_->setValue(msg.grid_search_parameters.cv_svm_width_step_size);
  width_base_box_->setValue(msg.grid_search_parameters.cv_svm_width_base);
  return true;
}

bool ModelSelectionGridSearchKernelGui::setRange(QComboBox* box, const std::string& range)
{
  int index = box->findText(QString(range.c_str()));
  if(index < 0)
  {
    return false;
  }
  box->setCurrentIndex(index);
  return true;
}

bool ModelSelectionGridSearchKernelGui::getRange(QComboBox* box, std::string& range)
{
  range = box->currentText().toStdString();
  return true;
}

void ModelSelectionGridSearchKernelGui::clearStatusReport()
{
  status_text_edit_->clear();
}

void ModelSelectionGridSearchKernelGui::setStatusReport(const std::string& status_report, const StatusReportMode mode)
{
  // ROS_INFO_STREAM(status_report);
  std::string status_string;
  const std::string debug_color = "#000000"; // black
  const std::string info_color = "#006400"; // dark green
  const std::string warn_color = "#FF6600"; // orange
  const std::string error_color = "#FF0000"; // red
  const std::string fatal_color = "#9400D3"; // dark violoet
  switch (mode)
  {
    case DEBUG:
    {
      status_string.assign("<span style=\"color: " + debug_color + "\">" + status_report + "</span><br />");
      break;
    }
    case INFO:
    {
      status_string.assign("<span style=\"color: " + info_color + "\">" + status_report + "</span><br />");

      break;
    }
    case WARN:
    {
      status_string.assign("<span style=\"color: " + warn_color + "\">" + status_report + "</span><br />");

      break;
    }
    case ERROR:
    {
      status_string.assign("<span style=\"color: " + error_color + "\">" + status_report + "</span><br />");

      break;
    }
    case FATAL:
    {
      status_string.assign("<span style=\"color: " + fatal_color + "\">" + status_report + "</span><br />");

      break;
    }
    default:
    {
      status_string.assign("<span style=\"color: " + warn_color + "\">(Unknown mode) " + status_report + "</span><br />");
      break;
    }
  }
  status_text_edit_->moveCursor(QTextCursor::End);
  status_text_edit_->insertHtml(QString(status_string.c_str()));
  // QScrollBar* scroll_bar = status_text_edit_->verticalScrollBar();
  // scroll_bar->setValue(scroll_bar->maximum());
  QCoreApplication::processEvents();
  repaint();
}

}

