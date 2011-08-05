/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    task_accumulator_io.h

 \author  Peter Pastor, Mrinal Kalakrishnan
 \date    Jul 14, 2010

 *********************************************************************/

#ifndef TASK_ACCUMULATOR_IO_H_
#define TASK_ACCUMULATOR_IO_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>
#include <map>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/AccumulatedTrialStatistics.h>

#include <task_recorder2_utilities/accumulator.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>
#include <task_recorder2_utilities/task_description_utilities.h>

// local includes

namespace task_recorder2_utilities
{

// default template parameters
template<class MessageType = task_recorder2_msgs::DataSample, class MessageTypeLabel = task_recorder2_msgs::DataSampleLabel> class TaskAccumulatorIO;

template<class MessageType, class MessageTypeLabel>
  class TaskAccumulatorIO
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
    typedef boost::shared_ptr<MessageTypeLabel const> MessageTypeLabelConstPtr;

    /*! Constructor
     */
    TaskAccumulatorIO(ros::NodeHandle node_handle) :
      node_handle_(node_handle), initialized_(false), trial_(0) {};
    /*! Destructor
     */
    virtual ~TaskAccumulatorIO() {};

    /*!
     * @param labeled_file_name
     * @param topic_name
     * @return True on success, otherwise False
     */
    bool initialize(const std::string labeled_file_name = std::string("/TaskRecorderManager/data_samples"),
                    const std::string topic_name = std::string("/TaskRecorderManager/data_sample_label"));

    /*!
     * @param description
     * @param directory_name
     * @return True on success, otherwise False
     */
    bool accumulate(const task_recorder2_msgs::Description& description,
                    const std::string directory_name = std::string(""));

    /*!
     */
    ros::NodeHandle node_handle_;
    std::string topic_name_;
    std::string labeled_file_name_;

  private:

    /*!
     */
    bool initialized_;
    int trial_;
    std::map<MessageType, MessageTypeLabel> messages_;
    task_recorder2_utilities::Accumulator accumulator_;
    std::string data_directory_name_;

  };

template<class MessageType, class MessageTypeLabel>
  bool TaskAccumulatorIO<MessageType, MessageTypeLabel>::initialize(const std::string labeled_file_name,
                                              const std::string topic_name)
  {
    labeled_file_name_ = labeled_file_name;
    topic_name_ = topic_name;
    ROS_DEBUG("Initializing task accumulator with topic >%s< labeling files with name >%s<.", topic_name_.c_str(), labeled_file_name_.c_str());

    std::string recorder_package_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_package_name", recorder_package_name));
    std::string recorder_data_directory_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_data_directory_name", recorder_data_directory_name));
    data_directory_name_ = getDirectoryPath(recorder_package_name, recorder_data_directory_name);
    ROS_VERIFY(checkAndCreateDirectories(data_directory_name_));

    return (initialized_ = true);
  }

template<class MessageType, class MessageTypeLabel>
  bool TaskAccumulatorIO<MessageType, MessageTypeLabel>::accumulate(const task_recorder2_msgs::Description& description,
                                                                    const std::string directory_name)
  {
    ROS_ASSERT_MSG(initialized_, "Task accumulator IO module is not initialized.");
    boost::filesystem::path absolute_data_directory_path = boost::filesystem::path(data_directory_name_ + getFileName(description));
    ROS_VERIFY(checkForDirectory(absolute_data_directory_path, false));
    std::string file_name = getPathNameIncludingTrailingSlash(absolute_data_directory_path);
    ROS_DEBUG("Setting absolute_data_directory_path to >%s<.", file_name.c_str());

    if(!directory_name.empty())
    {
      file_name.append(directory_name);
      ROS_VERIFY(checkForDirectory(file_name));
      usc_utilities::appendTrailingSlash(file_name);
    }

    ROS_DEBUG("Accumulating >%i< trials...", description.trial+1);
    std::vector<std_msgs::Header> headers;
    int num_messages = 0;
    accumulator_.clear();
    for (int i = 0; i <= description.trial; ++i)
    {
      std::string data_sample_file_name = file_name + getDataFileName(labeled_file_name_, i);
      std::vector<MessageType> messages;
      ROS_VERIFY(usc_utilities::FileIO<MessageType>::readFromBagFile(messages, labeled_file_name_, data_sample_file_name, false));
      ROS_VERIFY(accumulator_.accumulate(messages));
      if(i == 0)
      {
        num_messages = (int)messages.size();
        for (int j = 0; j < num_messages; ++j)
        {
          headers.push_back(messages[j].header);
        }
      }
      else
      {
        // error checking
        ROS_ASSERT(num_messages == (int)messages.size());
        for (int j = 0; j < (int)messages.size(); ++j)
        {
          // TODO: change this a bit...
          ROS_ASSERT_MSG(fabs(messages[j].header.stamp.toSec() - headers[j].stamp.toSec()) < 10e-4, "Headers on sample >%i< do not match. Stamps are >%f/%li< and >%f/%li<.",
              j, messages[j].header.stamp.toSec(), messages[j].header.stamp.toNSec(), headers[j].stamp.toSec(), headers[j].stamp.toNSec());
        }
      }
    }

    std::string stat_file_name = file_name + getStatFileName(labeled_file_name_);
    std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> accumulated_trial_statistics;
    ROS_VERIFY(accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
    ROS_ASSERT_MSG((int)accumulated_trial_statistics.size() == num_messages,
                   "Number of accumulated trial statistics >%i< does not match number of messages >%i<.",
                   (int)accumulated_trial_statistics.size(), num_messages);
    for (int i = 0; i < (int)accumulated_trial_statistics.size(); ++i)
    {
      accumulated_trial_statistics[i].description = description;
      accumulated_trial_statistics[i].header = headers[i];
    }
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::AccumulatedTrialStatistics>::writeToBagFileWithTimeStamps(accumulated_trial_statistics, "/accumulated_trial_statistics", stat_file_name, false));

    return true;
  }


}

#endif /* TASK_ACCUMULATOR_IO_H_ */
