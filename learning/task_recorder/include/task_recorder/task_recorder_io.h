/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \file    task_recorder_io.h

 \author  Peter Pastor
 \date    Jul 14, 2010

 **********************************************************************/

#ifndef TASK_RECORDER1_TASK_RECORDER_IO_H_
#define TASK_RECORDER1_TASK_RECORDER_IO_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <rosbag/bag.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <boost/filesystem.hpp>

// local includes
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder
{

template<class MessageType>
  class TaskRecorderIO
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    TaskRecorderIO() {};
    virtual ~TaskRecorderIO() {};

    /*!
     * @param node_handle
     * @param topic_name
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle,
                    const std::string& topic_name);

    /*!
     * @param id
     */
    void setId(const int id);

    /*!
     * @return
     */
    bool writeRecordedData();
    bool writeRawData();

    /*!
     * @return
     */
    bool writeStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics);

    /*!
     */
    ros::NodeHandle node_handle_;
    std::string topic_name_;

    /*!
     */
    std::vector<MessageType> messages_;

  private:

    int id_;
    int trial_;

    std::string data_directory_name_;

    boost::filesystem::path absolute_data_directory_path_;
    bool checkForDirectory();

  };

template<class MessageType>
  bool TaskRecorderIO<MessageType>::initialize(ros::NodeHandle& node_handle,
                                               const std::string& topic_name)
  {
    node_handle_ = node_handle;

    topic_name_ = topic_name;
    ROS_INFO("Initializing task recorder for topic named %s.", topic_name_.c_str());

    std::string package_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "package_name", package_name));
    std::string package_path = ros::package::getPath(package_name);
    usc_utilities::appendTrailingSlash(package_path);

    ROS_VERIFY(usc_utilities::read(node_handle_, "data_directory_name", data_directory_name_));
    usc_utilities::appendTrailingSlash(data_directory_name_);

    data_directory_name_.assign(package_path + data_directory_name_);

    try
    {
      boost::filesystem::create_directories(data_directory_name_);
    }
    catch (std::exception p)
    {
      ROS_ERROR("Data directory %s could not be created.", data_directory_name_.c_str());
      return false;
    }

    return true;
  }

template<class MessageType>
  void TaskRecorderIO<MessageType>::setId(const int id)
  {
    id_ = id;

    // check whether directory exists, if not, create it
    absolute_data_directory_path_ = boost::filesystem::path(data_directory_name_ + FILE_NAME_DATA_TRUNK + getString(id_));
    checkForDirectory();

    ROS_VERIFY(getTrialId(absolute_data_directory_path_, trial_, topic_name_));
    ROS_VERIFY(checkForCompleteness(absolute_data_directory_path_, trial_, topic_name_));

  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::checkForDirectory()
  {
    // check for the directory, if it does not exist -> create it.
    if (boost::filesystem::exists(absolute_data_directory_path_))
    {
      return true;
    }
    else
    {
      if (!boost::filesystem::create_directory(absolute_data_directory_path_))
      {
        ROS_ERROR_STREAM("Could not create directory " << absolute_data_directory_path_.filename() << " :" << std::strerror(errno));
        return false;
      }
    }
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRecordedData()
  {

    std::string file_name = getPathNameIncludingTrailingSlash(absolute_data_directory_path_) + getDataFileName(topic_name_, trial_);

    ROS_INFO("Writing data to >%s<.", file_name.c_str());
    try
    {
      rosbag::Bag bag;
      bag.open(file_name, rosbag::bagmode::Write);
      for (int i = 0; i < static_cast<int> (messages_.size()); ++i)
      {
        bag.write(topic_name_, messages_[i].header.stamp, messages_[i]);
      }
      bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
      ROS_ERROR("Problem when writing to bag file named %s.", file_name.c_str());
      return false;
    }

    if (!incrementTrialCounterFile(absolute_data_directory_path_, topic_name_))
    {
      return false;
    }

    ROS_VERIFY(getTrialId(absolute_data_directory_path_, trial_, topic_name_));
    ROS_VERIFY(checkForCompleteness(absolute_data_directory_path_, trial_, topic_name_));
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRawData()
  {

    std::string directory_name = getPathNameIncludingTrailingSlash(absolute_data_directory_path_) + std::string("raw");
    boost::filesystem::path directory_path = boost::filesystem::path(directory_name);
    if (!boost::filesystem::exists(directory_path))
    {
      // ROS_INFO("Creating %s", directory_name.c_str());
      if (!boost::filesystem::create_directory(directory_path))
      {
        ROS_ERROR_STREAM("Could not create directory " << directory_path.filename() << " :" << std::strerror(errno));
        return false;
      }
    }

    std::string file_name = directory_name + std::string("/") + getDataFileName(topic_name_, trial_);
    // ROS_INFO("Writing %s", file_name.c_str());

    return usc_utilities::FileIO<MessageType>::writeToBagFileWithTimeStamps(messages_, topic_name_, file_name);

//    try
//    {
//      rosbag::Bag bag;
//      bag.open(file_name, rosbag::bagmode::Write);
//      for (int i = 0; i < static_cast<int> (messages_.size()); ++i)
//      {
//        bag.write(topic_name_, messages_[i].header.stamp, messages_[i]);
//      }
//      bag.close();
//    }
//    catch (rosbag::BagIOException ex)
//    {
//      ROS_ERROR("Problem when writing to bag file named %s.", file_name.c_str());
//      return false;
//    }

    // ROS_INFO("Writing done");
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
  {
    std::string file_name = getPathNameIncludingTrailingSlash(absolute_data_directory_path_) + getStatFileName(topic_name_, trial_);
    try
    {
      rosbag::Bag bag;
      bag.open(file_name, rosbag::bagmode::Write);
      for (int i = 0; i < static_cast<int> (vector_of_accumulated_trial_statistics.size()); ++i)
      {
        std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics = vector_of_accumulated_trial_statistics[i];
        for (int j = 0; j < static_cast<int> (accumulated_trial_statistics.size()); ++j)
        {
          accumulated_trial_statistics[j].id = id_;
          bag.write(topic_name_, messages_[j].header.stamp, accumulated_trial_statistics[j]);
        }
      }
      bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
      ROS_ERROR("Problem when writing to bag file named %s : %s", file_name.c_str(), ex.what());
      return false;
    }
    return true;
  }

}

#endif /* TASK_RECORDER_IO_H_ */
