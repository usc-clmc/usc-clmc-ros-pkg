/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		svm_parameters.cpp

  \author	Peter Pastor
  \date		Jul 21, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/file_io.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <shogun/machine/Machine.h>
#include <shogun/kernel/Kernel.h>

// local includes
#include <task_event_detector/svm_parameters.h>

namespace task_event_detector
{

static const std::string SVM_PARAMETER_TOPIC_NAME = "/SVMParameters";
static const std::string SVM_PARAMETER_BAGFILE_NAME = "svm_parameter.bag";

SVMParameters::SVMParameters() :
  initialized_(false)
{
  setDefault();
}

void SVMParameters::setDefault()
{
  msg_.svm_lib = shogun::CT_LIBSVM;
  msg_.kernel_type = shogun::K_LINEAR;
  msg_.kernel_width = 0.0;
  msg_.svm_c = 0.0;
  msg_.svm_eps = 0.0;
  msg_.num_variables = 0;
  msg_.variable_names.clear();
  msg_.num_data_samples = 0;
  msg_.classification_boundary = 0.0;
}

bool SVMParameters::set(task_event_detector::SVMParametersMsg& parameters)
{
  msg_ = parameters;
  return (initialized_ = true);
}

task_event_detector::SVMParametersMsg SVMParameters::get() const
{
  ROS_ASSERT(initialized_);
  return msg_;
}

bool SVMParameters::load(const std::string& directory_name)
{
  ROS_VERIFY(usc_utilities::FileIO<SVMParametersMsg>::readFromBagFile(msg_, SVM_PARAMETER_TOPIC_NAME, directory_name + SVM_PARAMETER_BAGFILE_NAME, false));
  return (initialized_ = true);
}

bool SVMParameters::save(const std::string& directory_name) const
{
  ROS_ASSERT_MSG(initialized_, "SVMParameters not initialized. Cannot save then to file.");
  return usc_utilities::FileIO<SVMParametersMsg>::writeToBagFile(msg_, SVM_PARAMETER_TOPIC_NAME, directory_name + SVM_PARAMETER_BAGFILE_NAME);
}

bool SVMParameters::read(ros::NodeHandle node_handle)
{
  ROS_VERIFY(usc_utilities::read(node_handle, "svm_lib", msg_.svm_lib));
  if(msg_.svm_lib == shogun::CT_LIBSVM)
  {

  }
  else if(msg_.svm_lib == shogun::CT_LIGHT)
  {

  }
  else if(msg_.svm_lib == shogun::CT_LIGHTONECLASS)
  {

  }
  else if(msg_.svm_lib == shogun::CT_LIBSVMONECLASS)
  {

  }
  else
  {
    ROS_ERROR("Unknown SVM lib id >%i<.", msg_.svm_lib);
    return false;
  }

  ROS_VERIFY(usc_utilities::read(node_handle, "kernel_type", msg_.kernel_type));
  if(msg_.kernel_type == shogun::K_LINEAR)
  {

  }
  else if(msg_.kernel_type == shogun::K_GAUSSIAN)
  {
    ROS_VERIFY(usc_utilities::read(node_handle, "kernel_width", msg_.kernel_width));
  }
  else
  {
    ROS_ERROR("Unknown kernel type >%i<.", msg_.kernel_type);
    return false;
  }

  ROS_VERIFY(usc_utilities::read(node_handle, "svm_c", msg_.svm_c));
  ROS_VERIFY(usc_utilities::read(node_handle, "svm_eps", msg_.svm_eps));
  return (initialized_ = true);
}

}
