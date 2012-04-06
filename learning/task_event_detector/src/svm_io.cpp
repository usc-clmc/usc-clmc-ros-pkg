/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		svm_io.cpp

  \author	Peter Pastor
  \date		Jul 8, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_utilities/task_recorder_utilities.h>
#include <task_recorder2_utilities/task_description_utilities.h>

// local includes
#include <task_event_detector/svm_io.h>

using namespace task_recorder2_utilities;

namespace task_event_detector
{

SVMIO::SVMIO(ros::NodeHandle node_handle) : node_handle_(node_handle)
{
  std::string svm_package_name;
  ROS_VERIFY(usc_utilities::read(node_handle_, "svm_package_name", svm_package_name));
  std::string svm_data_directory_name;
  ROS_VERIFY(usc_utilities::read(node_handle_, "svm_data_directory_name", svm_data_directory_name));
  data_directory_name_ = getDirectoryPath(svm_package_name, svm_data_directory_name);
  ROS_VERIFY(checkAndCreateDirectories(data_directory_name_));
  ROS_DEBUG("Setting SVM data directory name to >%s<.", data_directory_name_.c_str());
}

bool SVMIO::write(const task_recorder2_msgs::Description& description,
                  SVMClassifierPtr& svm_classifier) const
{
  boost::filesystem::path path = boost::filesystem::path(data_directory_name_ + getFileName(description));
  ROS_DEBUG("Writing SVM to file >%s<.", path.file_string().c_str());
  // check whether directory exists, if not, create it
  if(!svm_classifier->isInitialized())
  {
    ROS_ERROR("SVM is not initialized. Cannot write to >%s<.", path.file_string().c_str());
    return false;
  }
  if(!svm_classifier->isTrained())
  {
    ROS_ERROR("SVM is not learned. Cannot write to >%s<.", path.file_string().c_str());
    return false;
  }
  ROS_VERIFY(checkForDirectory(path));
  return svm_classifier->save(path.file_string());
}

bool SVMIO::read(const task_recorder2_msgs::Description& description,
                 SVMClassifierPtr& svm_classifier) const
{
  svm_classifier.reset(new SVMClassifier());
  if(!svm_classifier->read(node_handle_))
  {
    ROS_ERROR("Could not initialize SVM from node handle.");
    return false;
  }
  boost::filesystem::path path = boost::filesystem::path(data_directory_name_ + getFileName(description));
  if(!checkForDirectory(path, false))
  {
    ROS_ERROR("Directory >%s< does not exist, cannot initialize SVM.", path.file_string().c_str());
    return false;
  }
  return svm_classifier->load(path.file_string());
}

bool SVMIO::getList(std::vector<std::string>& descriptions)
{
  boost::filesystem::path path = boost::filesystem::path(data_directory_name_);
  return task_recorder2_utilities::getDirectoryList(path, descriptions);
}

}
