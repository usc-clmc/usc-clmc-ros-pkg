/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		label_gui_client.cpp

  \author	Peter Pastor
  \date		Aug 5, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/services.h>

#include <task_recorder2_msgs/SetLabel.h>
#include <task_recorder2_msgs/SetLastTrialIds.h>

// local includes
#include <task_label_gui/label_gui_client.h>

namespace task_label_gui
{

LabelGuiClient::LabelGuiClient(ros::NodeHandle node_handle) :
    node_handle_(node_handle)
{
  set_last_trial_ids_service_client_ = node_handle_.serviceClient<task_recorder2_msgs::SetLabel>("setLastTrialIds");
  set_label_service_client_ = node_handle_.serviceClient<task_recorder2_msgs::SetLabel>("setLabel");
  record_and_label_service_client_ = node_handle_.serviceClient<task_recorder2_msgs::SetLabel>("recordAndLabel");
}

bool LabelGuiClient::setTrialIds(std::vector<task_recorder2_msgs::Description>& descriptions,
                                 const bool wait_for_label)
{
  if(!getenv("ARM_LABEL_GRASPS"))
  {
    ROS_DEBUG("ARM_LABEL_GRASPS is not set. Skipping labeling...");
    return false;
  }
  if(wait_for_label)
  {
    usc_utilities::waitFor(set_last_trial_ids_service_client_);
  }
  if(!usc_utilities::isReady(set_last_trial_ids_service_client_))
  {
    ROS_WARN("Label gui is not running, skipping setting trial ids.");
  }
  else
  {
    task_recorder2_msgs::SetLastTrialIds::Request request;
    request.descriptions = descriptions;
    task_recorder2_msgs::SetLastTrialIds::Response response;
    ROS_VERIFY(set_last_trial_ids_service_client_.call(request, response));
    ROS_ASSERT_MSG(response.descriptions.size() == descriptions.size(),
                   "Number of responded descriptions >%i< should equal number of requested descriptions >%i<.", (int)response.descriptions.size(), (int)descriptions.size());
  }
  return true;
}

bool LabelGuiClient::setLabel(const std::vector<task_recorder2_msgs::Description>& descriptions,
                              std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels,
                              const bool wait_for_label)
{
  if(!getenv("ARM_LABEL_GRASPS"))
  {
    ROS_DEBUG("ARM_LABEL_GRASPS is not set. Skipping labeling...");
    return false;
  }
  if(wait_for_label)
  {
    usc_utilities::waitFor(set_label_service_client_);
  }
  if(!usc_utilities::isReady(set_label_service_client_))
  {
    ROS_WARN("Label gui is not running, skipping labeling.");
  }
  else
  {
    task_recorder2_msgs::SetLabel::Request request;
    request.descriptions = descriptions;
    task_recorder2_msgs::SetLabel::Response response;
    ROS_VERIFY(set_label_service_client_.call(request, response));
    if((response.return_code == task_recorder2_msgs::SetLabel::Response::FAILED)
        || (response.return_code == task_recorder2_msgs::SetLabel::Response::ABORTED))
    {
      return false;
    }
    data_sample_labels = response.labels;
  }
  return true;
}

bool LabelGuiClient::recordAndLabel(const std::vector<task_recorder2_msgs::Description>& descriptions,
                                    std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels,
                                    const bool wait_for_label)
{
  if(!getenv("ARM_LABEL_GRASPS"))
  {
    ROS_DEBUG("ARM_LABEL_GRASPS is not set. Skipping labeling...");
    return false;
  }
  if(wait_for_label)
  {
    usc_utilities::waitFor(record_and_label_service_client_);
  }
  if(!usc_utilities::isReady(record_and_label_service_client_))
  {
    ROS_WARN("Label gui is not running, skipping recording and labeling.");
  }
  else
  {
    task_recorder2_msgs::SetLabel::Request request;
    request.descriptions = descriptions;
    task_recorder2_msgs::SetLabel::Response response;
    ROS_VERIFY(record_and_label_service_client_.call(request, response));
    ROS_INFO_STREAM_COND(!response.info.empty(), response.info);
    if((response.return_code == task_recorder2_msgs::SetLabel::Response::FAILED)
        || (response.return_code == task_recorder2_msgs::SetLabel::Response::ABORTED))
    {
      return false;
    }
    data_sample_labels = response.labels;
  }
  return true;
}

}
