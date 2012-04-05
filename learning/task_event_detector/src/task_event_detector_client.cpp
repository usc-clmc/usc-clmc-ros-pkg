/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_event_detector_client.cpp

  \author	Peter Pastor
  \date		Jul 8, 2011

 *********************************************************************/

// system includes
#include <task_recorder2_msgs/DetectedEvents.h>
#include <task_recorder2_msgs/BinaryLabel.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>

#include <usc_utilities/assert.h>

#include <task_recorder2_msgs/SetupDetector.h>
#include <task_recorder2_msgs/DetectedEvents.h>

// local includes
#include <task_event_detector/task_event_detector_client.h>

namespace task_event_detector
{

TaskEventDetectorClient::TaskEventDetectorClient(ros::NodeHandle node_handle)
  : initialized_(false), node_handle_(node_handle)
{
  load_service_client_ = node_handle_.serviceClient<task_recorder2_msgs::SetupDetector> (std::string("/TaskEventDetector/load"));
  detect_service_client_ = node_handle_.serviceClient<task_recorder2_msgs::DetectedEvents> (std::string("/TaskEventDetector/detect"));
}

bool TaskEventDetectorClient::doesSVMExist(const task_recorder2_msgs::Description& description)
{
  std::vector<std::string> descriptions;
  if(!getListOfSVMs(descriptions))
  {
    ROS_ERROR("Could not get list of SVMs.");
    return false;
  }
  bool svm_exists = false;
  for (int i = 0; !svm_exists && i < (int)descriptions.size(); ++i)
  {
    if(descriptions[i].compare(task_recorder2_utilities::getFileName(description)) == 0)
    {
      svm_exists = true;
    }
  }
  return svm_exists;
}

bool TaskEventDetectorClient::setDescription(const task_recorder2_msgs::Description& description)
{
  if(!doesSVMExist(description))
  {
    ROS_WARN("Check for SVM with description >%s< failed.", task_recorder2_utilities::getFileName(description).c_str());
    return (initialized_ = false);
  }
  task_recorder2_msgs::SetupDetector::Request request;
  request.description = description;
  task_recorder2_msgs::SetupDetector::Response response;
  if (!load_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when loading SVM for >%s<.", task_recorder2_utilities::getFileName(description).c_str());
    return (initialized_ = false);
  }
  if(response.return_code != task_recorder2_msgs::SetupDetector::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< did not return successfully.", load_service_client_.getService().c_str());
    ROS_ERROR_STREAM_COND(!response.info.empty(), response.info);
    return false;
  }
  ROS_INFO_STREAM_COND(!response.info.empty(), response.info);
  description_ = description;
  return (initialized_ = true);
}

bool TaskEventDetectorClient::didEventOccur(bool& event_occured,
                                            const bool save_data_samples,
                                            const int num_data_samples)
{
  task_recorder2_msgs::DataSampleLabel label;
  if(!checkForEvent(label, save_data_samples, num_data_samples))
  {
    return false;
  }
  ROS_ASSERT(label.type == task_recorder2_msgs::DataSampleLabel::BINARY_LABEL);
  event_occured = (label.binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED);
  return true;
}

bool TaskEventDetectorClient::getLabels(const task_recorder2_msgs::Description& svm_description,
                                        const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                                        std::vector<task_recorder2_msgs::DataSampleLabel>& predicted_labels)
{
  if(!doesSVMExist(svm_description))
  {
    ROS_ERROR("SVM with description >%s< does not exist.", task_recorder2_utilities::getFileName(svm_description).c_str());
    return false;
  }
  if(!Detector::load(svm_description))
  {
    ROS_ERROR("Could not load SVM from description >%s<.", task_recorder2_utilities::getFileName(svm_description).c_str());
    return false;
  }
  return predict(data_samples, predicted_labels);
}

bool TaskEventDetectorClient::checkForEvent(task_recorder2_msgs::DataSampleLabel& label,
                                            const bool save_data_samples,
                                            const int num_data_samples)
{
  if(!initialized_)
  {
    ROS_ERROR("Task event detector client is not initialized. You need to specify a description using setDescription.");
    return false;
  }
  if(!task_recorder_manager_client_.checkForServices())
  {
    ROS_ERROR("Not all task recorder manager services are online. Cannot check for event.");
    return false;
  }
  ROS_VERIFY(task_recorder_manager_client_.startStreaming());
  task_recorder2_msgs::DetectedEvents::Request request;
  request.description = description_;
  request.method = task_recorder2_msgs::DetectedEvents::Request::AVERAGING;
  request.num_data_samples = num_data_samples;
  request.save_data_samples = save_data_samples;
  task_recorder2_msgs::DetectedEvents::Response response;
  if (!detect_service_client_.call(request, response))
  {
    ROS_ERROR("Problems when detecting events.");
    return false;
  }
  if(response.return_code != task_recorder2_msgs::DetectedEvents::Response::SERVICE_CALL_SUCCESSFUL)
  {
    ROS_ERROR("Service >%s< did not return successfully.", detect_service_client_.getService().c_str());
    ROS_ERROR_STREAM_COND(!response.info.empty(), response.info);
    return false;
  }
  ROS_INFO_STREAM_COND(!response.info.empty(), response.info);
  ROS_VERIFY(task_recorder_manager_client_.interruptRecording());
  label = response.label;
  return true;
}

bool TaskEventDetectorClient::labelSample(const std::vector<task_recorder2_msgs::Description>& descriptions,
                                          std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels,
                                          const bool wait_for_label)
{
  return label_gui_client_.recordAndLabel(descriptions, data_sample_labels, wait_for_label);
}

}
