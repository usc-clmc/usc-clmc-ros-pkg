/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    task_labeler_io.h

 \author  Peter Pastor, Mrinal Kalakrishnan
 \date    Jul 14, 2010

 *********************************************************************/

#ifndef TASK_LABELER_IO_H_
#define TASK_LABELER_IO_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/AccumulatedTrialStatistics.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>
#include <task_recorder2_utilities/task_description_utilities.h>

// local includes

namespace task_recorder2
{

// default template parameters
template<class MessageType = task_recorder2_msgs::DataSampleLabel> class TaskLabelerIO;

template<class MessageType>
  class TaskLabelerIO
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    TaskLabelerIO(ros::NodeHandle node_handle) :
      node_handle_(node_handle), initialized_(false) {};
    /*! Destructor
     */
    virtual ~TaskLabelerIO() {};

    /*!
     * @param labeled_file_name
     * @param topic_name
     * @return True on success, otherwise False
     */
    bool initialize(const std::string labeled_file_name = std::string("/TaskRecorderManager/data_samples"),
                    const std::string topic_name = std::string("/TaskRecorderManager/data_sample_label"));

    /*!
     * @param description
     * @param label
     * @param directory_name
     * @return True on success, otherwise False
     */
    bool appendLabel(const task_recorder2_msgs::Description& description,
                     const MessageType& label,
                     const std::string directory_name = std::string(""));
    bool appendResampledLabel(const task_recorder2_msgs::Description& description,
                              const MessageType& label);
    bool appendRawLabel(const task_recorder2_msgs::Description& description,
                        const MessageType& label);

    /*!
     */
    ros::NodeHandle node_handle_;
    std::string topic_name_;
    std::string labeled_file_name_;

    /*!
     */
    bool write_out_raw_data_;
    bool write_out_resampled_data_;

  private:

    /*!
     */
    bool initialized_;
    MessageType messages_;
    std::string data_directory_name_;

  };

template<class MessageType>
  bool TaskLabelerIO<MessageType>::initialize(const std::string labeled_file_name,
                                              const std::string topic_name)
  {
    labeled_file_name_ = labeled_file_name;
    topic_name_ = topic_name;
    ROS_DEBUG("Initializing task labeler with topic >%s< labeling files with name >%s< in namespace >%s<.",
              topic_name_.c_str(), labeled_file_name_.c_str(), node_handle_.getNamespace().c_str());

    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_resampled_data", write_out_resampled_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_raw_data", write_out_raw_data_));

    std::string recorder_package_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_package_name", recorder_package_name));
    std::string recorder_data_directory_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_data_directory_name", recorder_data_directory_name));
    data_directory_name_ = task_recorder2_utilities::getDirectoryPath(recorder_package_name, recorder_data_directory_name);
    ROS_VERIFY(task_recorder2_utilities::checkAndCreateDirectories(data_directory_name_));

    return (initialized_ = true);
  }

template<class MessageType>
  bool TaskLabelerIO<MessageType>::appendLabel(const task_recorder2_msgs::Description& description,
                                               const MessageType& label,
                                               const std::string directory_name)
  {
    ROS_ASSERT_MSG(initialized_, "Task labeler IO module is not initialized.");
    boost::filesystem::path absolute_data_directory_path = boost::filesystem::path(data_directory_name_ + task_recorder2_utilities::getFileName(description));
    ROS_VERIFY(task_recorder2_utilities::checkForDirectory(absolute_data_directory_path, false));
    std::string file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(absolute_data_directory_path);
    ROS_DEBUG("Setting absolute_data_directory_path to >%s<.", file_name.c_str());

    if(!directory_name.empty())
    {
      file_name.append(directory_name);
      ROS_VERIFY(task_recorder2_utilities::checkForDirectory(file_name));
      usc_utilities::appendTrailingSlash(file_name);
    }
    file_name.append(task_recorder2_utilities::getDataFileName(labeled_file_name_, description.trial));
    ROS_DEBUG("Appending to file >%s< the topic >%s<.", file_name.c_str(), topic_name_.c_str());
    ROS_VERIFY(usc_utilities::FileIO<MessageType>::appendToBagFileWithTimeStamp(label, ros::TIME_MIN, topic_name_, file_name, false));
    return true;
  }
template<class MessageType>
  bool TaskLabelerIO<MessageType>::appendResampledLabel(const task_recorder2_msgs::Description& description,
                                                        const MessageType& label)
  {
    return appendLabel(description, label, "resampled");
  }
template<class MessageType>
  bool TaskLabelerIO<MessageType>::appendRawLabel(const task_recorder2_msgs::Description& description,
                                                  const MessageType& label)
  {
    return appendLabel(description, label, "raw");
  }

}

#endif /* TASK_LABELER_IO_H_ */
