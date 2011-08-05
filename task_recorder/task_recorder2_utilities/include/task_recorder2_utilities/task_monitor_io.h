/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_monitor_io.h

  \author	Peter Pastor
  \date		Jun 22, 2011

 *********************************************************************/

#ifndef TASK_MONITOR_IO_H_
#define TASK_MONITOR_IO_H_

// system includes

// local includes
#include <vector>
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

namespace task_recorder2_utilities
{

// default template parameters
template<class MessageType = task_recorder2_msgs::DataSample, class MessageTypeLabel = task_recorder2_msgs::DataSampleLabel> class TaskMonitorIO;

template<class MessageType, class MessageTypeLabel>
  class TaskMonitorIO
  {

  public:

  typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
  typedef boost::shared_ptr<MessageTypeLabel const> MessageTypeLabelConstPtr;

    /*! Constructor
     */
    TaskMonitorIO(ros::NodeHandle node_handle) :
      node_handle_(node_handle), initialized_(false) {};
    /*! Destructor
     */
    virtual ~TaskMonitorIO() {};

    /*!
     * @param labeled_file_name
     * @param topic_name
     * @return True on success, otherwise False
     */
    bool initialize(const std::string labeled_file_name = std::string("/TaskRecorderManager/data_samples"),
                    const std::string topic_name = std::string("/TaskRecorderManager/data_sample_label"));

    /*!
     * @param description
     * @param data_samples
     * @param data_label
     * @return True on success, otherwise False
     */
    bool readData(const task_recorder2_msgs::Description& description,
                  std::vector<MessageType>& data_samples,
                  MessageTypeLabel& data_label);

    /*!
     * @param description
     * @param data_samples
     * @param data_label
     * @return True on success, otherwise False
     */
    bool readAllData(const task_recorder2_msgs::Description& description,
                     std::vector<std::vector<MessageType> >& all_data_samples,
                     std::vector<MessageTypeLabel>& all_data_label);

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
    std::string data_directory_name_;

  };

template<class MessageType, class MessageTypeLabel>
  bool TaskMonitorIO<MessageType, MessageTypeLabel>::initialize(const std::string labeled_file_name,
                                                                const std::string topic_name)
  {
    labeled_file_name_ = labeled_file_name;
    topic_name_ = topic_name;
    ROS_DEBUG("Initializing task labeler with topic >%s< labeling files with name >%s<.", topic_name_.c_str(), labeled_file_name_.c_str());

    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_resampled_data", write_out_resampled_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_raw_data", write_out_raw_data_));

    std::string recorder_package_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_package_name", recorder_package_name));
    std::string recorder_data_directory_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_data_directory_name", recorder_data_directory_name));
    data_directory_name_ = getDirectoryPath(recorder_package_name, recorder_data_directory_name);
    ROS_VERIFY(checkAndCreateDirectories(data_directory_name_));

    return (initialized_ = true);
  }

template<class MessageType, class MessageTypeLabel>
  bool TaskMonitorIO<MessageType, MessageTypeLabel>::readData(const task_recorder2_msgs::Description& description,
                                                              std::vector<MessageType>& data_samples,
                                                              MessageTypeLabel& data_label)
  {
    ROS_ASSERT_MSG(initialized_, "Task labeler IO module is not initialized.");
    boost::filesystem::path absolute_data_directory_path = boost::filesystem::path(data_directory_name_ + getFileName(description));
    ROS_VERIFY(checkForDirectory(absolute_data_directory_path, false));
    std::string file_name = getPathNameIncludingTrailingSlash(absolute_data_directory_path);
    ROS_DEBUG("Setting absolute_data_directory_path to >%s<.", file_name.c_str());

    file_name.append(getDataFileName(labeled_file_name_, description.trial));
    ROS_VERIFY(usc_utilities::FileIO<MessageType>::readFromBagFile(data_samples, labeled_file_name_, file_name, false));
    ROS_VERIFY(usc_utilities::FileIO<MessageTypeLabel>::readFromBagFile(data_label, topic_name_, file_name, false));
    ROS_INFO("Read >%i< data samples and 1 label.", (int)data_samples.size());
    return true;
  }

template<class MessageType, class MessageTypeLabel>
  bool TaskMonitorIO<MessageType, MessageTypeLabel>::readAllData(const task_recorder2_msgs::Description& description,
                                                                 std::vector<std::vector<MessageType> >& all_data_samples,
                                                                 std::vector<MessageTypeLabel>& all_data_label)
  {
    ROS_ASSERT_MSG(initialized_, "Task labeler IO module is not initialized.");

    // get number of trials
    boost::filesystem::path absolute_data_directory_path = boost::filesystem::path(data_directory_name_ + getFileName(description));
    ROS_VERIFY(checkForDirectory(absolute_data_directory_path, false));

    int number_of_trials;
    ROS_VERIFY(getTrialId(absolute_data_directory_path, number_of_trials, labeled_file_name_));
    ROS_INFO("There are >%i< trials.", number_of_trials);

    all_data_samples.clear();
    all_data_label.clear();

    task_recorder2_msgs::Description tmp_description = description;
    for (int i = 0; i < number_of_trials; ++i)
    {
      std::vector<MessageType> data_samples;
      MessageTypeLabel data_label;
      tmp_description.trial = i;
      ROS_VERIFY(readData(tmp_description, data_samples, data_label));

      all_data_label.push_back(data_label);
      all_data_samples.push_back(data_samples);
    }
    return true;
  }


}


#endif /* TASK_MONITOR_IO_H_ */
