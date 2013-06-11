/*
 * bag_file_recorder_manager.cpp
 *
 *  Created on: Jun 10, 2013
 *      Author: pastor
 */

#include <string>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <task_recorder2_utilities/data_sample_utilities.h>
#include <task_recorder2/bag_file_recorder_manager.h>

#include <tf/tfMessage.h>
#include <task_recorder2_msgs/tfMessageStamped.h>
#include <task_recorder2/msg_header_utility.h>

namespace task_recorder2
{

BagFileRecorderManager::BagFileRecorderManager()
  : initialized_(false)
{
  clients_.clear();
}

bool BagFileRecorderManager::initialize(ros::NodeHandle node_handle)
{
  std::vector<std::string> topic_names;
  ROS_VERIFY(usc_utilities::read(node_handle, "topic_names", topic_names));

  for (unsigned int i = 0; i < topic_names.size(); ++i)
  {
    ROS_WARN("Creating >%s< bag file recorder.", topic_names[i].c_str());
    if(topic_names[i].compare("/tf") == 0)
    {
      boost::shared_ptr<task_recorder2::BagFileRecorderClient<tf::tfMessage, task_recorder2_msgs::tfMessageStamped, task_recorder2::TFHeaderExtractor<tf::tfMessage> > > recorder;
      recorder.reset(new task_recorder2::BagFileRecorderClient<tf::tfMessage, task_recorder2_msgs::tfMessageStamped, task_recorder2::TFHeaderExtractor<tf::tfMessage> >(topic_names[i]));
      clients_.push_back(recorder);
    }
    else
    {
      ROS_ERROR("Unkown recorder client >%s<.", topic_names[i].c_str());
      return false;
    }
  }
  return (initialized_ = true);
}

bool BagFileRecorderManager::startRecording(const std::string& description, const int id)
{
  ROS_INFO("Starting to record bagfiles for >%s< and id >%i<.", description.c_str(), id);
  ROS_ASSERT(initialized_);
  bool success = true;
  for (unsigned int i = 0; success && i < clients_.size(); ++i)
  {
    ROS_WARN_COND(clients_[i]->isRecording(), "Already recording... starting over.");
    success = clients_[i]->startRecording(description, id);
  }
  ROS_INFO("Done starting to record bagfiles for >%s< and id >%i<", description.c_str(), id);
  return success;
}

bool BagFileRecorderManager::stopRecording(const ros::Time& crop_start_time, const ros::Time& crop_end_time)
{
  ROS_INFO("Stopping to record bagfiles");
  ROS_ASSERT(initialized_);
  bool success = true;
  for (unsigned int i = 0; success && i < clients_.size(); ++i)
  {
    success = clients_[i]->stopRecording(crop_start_time, crop_end_time);
  }
  return success;
}


}


