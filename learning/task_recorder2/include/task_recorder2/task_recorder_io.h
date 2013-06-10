/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    task_recorder_io.h

 \author  Peter Pastor, Mrinal Kalakrishnan
 \date    Jul 14, 2010

 *********************************************************************/

#ifndef TASK_RECORDER_IO_H_
#define TASK_RECORDER_IO_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>
#include <boost/scoped_ptr.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/AccumulatedTrialStatistics.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>
#include <task_recorder2_utilities/task_description_utilities.h>

#include <dmp_lib/trajectory.h>

// local includes

namespace task_recorder2
{

static const int ROS_TIME_OFFSET = 1340100000;

// default template parameters
template<class MessageType = task_recorder2_msgs::DataSample> class TaskRecorderIO;

template<class MessageType>
  class TaskRecorderIO
  {

  public:

  typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    TaskRecorderIO(ros::NodeHandle node_handle) :
      node_handle_(node_handle), write_out_raw_data_(false), write_out_clmc_data_(false),
      write_out_resampled_data_(false), write_out_statistics_(false), initialized_(false)
    {
      ROS_DEBUG("Reserving memory for >%i< messages.", NUMBER_OF_INITIALLY_RESERVED_MESSAGES);
      messages_.reserve(NUMBER_OF_INITIALLY_RESERVED_MESSAGES);
    };

    /*! Destructor
     */
    virtual ~TaskRecorderIO() {};

    /*!
     * @param topic_name
     * @param prefix
     * @return True on success, otherwise False
     */
    bool initialize(const std::string& topic_name, const std::string prefix = "");

    /*!
     * @param description
     * @param directory_name
     */
    void setDescription(const task_recorder2_msgs::Description& description,
                        const std::string directory_name = std::string(""));
    void setResampledDescription(const task_recorder2_msgs::Description& description,
                        const std::string directory_name = std::string("resampled"));

    /*!
     * @return
     */
    task_recorder2_msgs::Description getDescription() const;

    /*!
     * @param directory_name
     * @param increment_trial_counter
     * @return True on success, otherwise False
     */
    bool writeRecordedData(const std::string directory_name, bool increment_trial_counter);
    bool writeResampledData();

    /*!
     * @param directory_name
     * @return True on success, otherwise False
     */
    bool writeRecordedDataToCLMCFile(const std::string directory_name = std::string(""));

    /*!
     * @return True on success, otherwise False
     */
    bool writeRecordedDataSamples();

    /*!
     * @param raw_directory_name
     * @return True on success, otherwise False
     */
    bool writeRawData(const std::string raw_directory_name);
    bool writeRawData();

    /*!
     * @return True on success, otherwise False
     */
    bool writeStatistics(std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics);

    /*!
     * @param description
     * @param abs_file_name
     * @return True on success, otherwise False
     */
    bool getAbsFileName(const task_recorder2_msgs::Description& description,
                        std::string& abs_file_name);


    /*!
     * @param description
     * @param data_samples
     * @return True on success, otherwise False
     */
    bool readDataSamples(const task_recorder2_msgs::Description& description,
                         std::vector<MessageType>& msgs);

    /*!
     * @param descriptions
     * @return True on success, otherwise False
     */
    bool getList(std::vector<std::string>& descriptions);

    /*!
     */
    ros::NodeHandle node_handle_;
    std::string topic_name_;
    std::string prefixed_topic_name_;

    /*!
     */
    std::vector<MessageType> messages_;
    bool write_out_raw_data_;
    bool write_out_clmc_data_;
    bool write_out_resampled_data_;
    bool write_out_statistics_;

  private:

    static const int NUMBER_OF_INITIALLY_RESERVED_MESSAGES = 20 * 300;

    /*!
     */
    // int trial_;
    bool initialized_;

    /*!
     */
    task_recorder2_msgs::Description description_;
    std::string data_directory_name_;

    /*!
     */
    boost::filesystem::path absolute_data_directory_path_;
    bool create_directories_;

  };

template<class MessageType>
  bool TaskRecorderIO<MessageType>::initialize(const std::string& topic_name,
                                               const std::string prefix)
  {
    topic_name_ = topic_name;
    prefixed_topic_name_ = topic_name;
    usc_utilities::removeLeadingSlash(prefixed_topic_name_);
    prefixed_topic_name_ = prefix + prefixed_topic_name_;
    usc_utilities::appendLeadingSlash(prefixed_topic_name_);

    ROS_INFO("Initializing task recorder >%s< for topic named >%s<.", prefixed_topic_name_.c_str(), topic_name_.c_str());

    node_handle_.param("create_directories", create_directories_, true);
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_resampled_data", write_out_resampled_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_raw_data", write_out_raw_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_clmc_data", write_out_clmc_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_statistics", write_out_statistics_));

    std::string recorder_package_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_package_name", recorder_package_name));
    std::string recorder_data_directory_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_data_directory_name", recorder_data_directory_name));
    data_directory_name_ = task_recorder2_utilities::getDirectoryPath(recorder_package_name, recorder_data_directory_name);
    ROS_VERIFY(task_recorder2_utilities::checkAndCreateDirectories(data_directory_name_));
    ROS_DEBUG("Setting TaskRecorderIO data directory name to >%s<.", data_directory_name_.c_str());

    return (initialized_ = true);
  }

template<class MessageType>
  void TaskRecorderIO<MessageType>::setDescription(const task_recorder2_msgs::Description& description,
                                                   const std::string directory_name)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    description_ = description;

    if(create_directories_)
    {
      // check whether directory exists, if not, create it
      absolute_data_directory_path_ = boost::filesystem::path(data_directory_name_ + task_recorder2_utilities::getFileName(description_));
      boost::filesystem::path path = absolute_data_directory_path_;
      if(!directory_name.empty())
      {
        path = boost::filesystem::path(absolute_data_directory_path_.directory_string() + std::string("/") + directory_name);
      }
      ROS_VERIFY(task_recorder2_utilities::checkForDirectory(path));
      ROS_VERIFY(task_recorder2_utilities::getTrialId(path, description_.trial, prefixed_topic_name_));
      ROS_VERIFY(task_recorder2_utilities::checkForCompleteness(path, description_.trial, prefixed_topic_name_));
      ROS_DEBUG("Setting trial to >%i<.", description_.trial);
    }
    else
    {
      boost::filesystem::path path = boost::filesystem::path(data_directory_name_);
      ROS_VERIFY(task_recorder2_utilities::checkForDirectory(path));
      absolute_data_directory_path_ = boost::filesystem::path(data_directory_name_ + task_recorder2_utilities::getBagFileName(description_));
    }
  }

template<class MessageType>
  void TaskRecorderIO<MessageType>::setResampledDescription(const task_recorder2_msgs::Description& description,
                                                            const std::string directory_name)
  {
    setDescription(description, directory_name);
  }

template<class MessageType>
  task_recorder2_msgs::Description TaskRecorderIO<MessageType>::getDescription() const
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    return description_;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRecordedData(const std::string directory_name,
                                                      bool increment_trial_counter)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");

    if(create_directories_)
    {
      std::string file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(absolute_data_directory_path_);
      boost::filesystem::path path = absolute_data_directory_path_;
      if (!directory_name.empty())
      {
        file_name.append(directory_name);
        path = boost::filesystem::path(absolute_data_directory_path_.directory_string() + std::string("/") + directory_name);
        ROS_VERIFY(task_recorder2_utilities::checkForDirectory(file_name));
        usc_utilities::appendTrailingSlash(file_name);
      }
      file_name.append(task_recorder2_utilities::getDataFileName(prefixed_topic_name_, description_.trial));
      ROS_VERIFY(usc_utilities::FileIO<MessageType>::writeToBagFileWithTimeStamps(messages_, topic_name_, file_name, false));
      if (increment_trial_counter)
      {
        ROS_VERIFY(task_recorder2_utilities::incrementTrialCounterFile(path, prefixed_topic_name_));
        ROS_VERIFY(task_recorder2_utilities::getTrialId(path, description_.trial, prefixed_topic_name_));
        ROS_VERIFY(task_recorder2_utilities::checkForCompleteness(path, description_.trial, prefixed_topic_name_));
      }
    }
    else
    {
      std::string file_name = absolute_data_directory_path_.file_string();
      ROS_VERIFY(usc_utilities::FileIO<MessageType>::writeToBagFileWithTimeStamps(messages_, topic_name_, file_name, false));
    }
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::getAbsFileName(const task_recorder2_msgs::Description& description,
                                                   std::string& abs_file_name)
  {
    boost::filesystem::path path = boost::filesystem::path(data_directory_name_ + task_recorder2_utilities::getFileName(description));
    abs_file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(path);
    abs_file_name.append(task_recorder2_utilities::getDataFileName(prefixed_topic_name_, description.trial));
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::readDataSamples(const task_recorder2_msgs::Description& description,
                                                    std::vector<MessageType>& msgs)
  {
    std::string abs_file_name;
    if (!getAbsFileName(description, abs_file_name))
    {
      ROS_ERROR("Could not get file name of >%s<.", task_recorder2_utilities::getFileName(description).c_str());
      return false;
    }
    if (!usc_utilities::FileIO<MessageType>::readFromBagFile(msgs, topic_name_, abs_file_name, false))
    {
      ROS_ERROR("Could not read data samples in >%s<.", abs_file_name.c_str());
      return false;
    }
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRecordedDataToCLMCFile(const std::string directory_name)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    ROS_ASSERT_MSG(!messages_.empty(), "Messages are empty. Cannot write anything to CLMC file.");
    if(create_directories_)
    {
      std::string file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(absolute_data_directory_path_);
      boost::filesystem::path path = absolute_data_directory_path_;
      if (!directory_name.empty())
      {
        file_name.append(directory_name);
        path = boost::filesystem::path(absolute_data_directory_path_.directory_string() + std::string("/") + directory_name);
        ROS_VERIFY(task_recorder2_utilities::checkForDirectory(file_name));
        usc_utilities::appendTrailingSlash(file_name);
      }
      std::string clmc_file_name;
      ROS_VERIFY(task_recorder2_utilities::setCLMCFileName(clmc_file_name, description_.trial - 1));
      file_name.append(clmc_file_name);

      const int trajectory_length = (int)messages_.size();
      double trajectory_duration = (messages_[trajectory_length-1].header.stamp - messages_[0].header.stamp).toSec();
      if(trajectory_length == 1) // TODO: think about this again...
      {
        ROS_WARN("Only 1 data sample contained when writing out clmc data file.");
        trajectory_duration = 1.0;
      }
      ROS_ASSERT_MSG(trajectory_duration > 0.0, "Trajectory duration >%f< of trajectory named >%s< must be possitive.", trajectory_duration, file_name.c_str());
      const double SAMPLING_FREQUENCY = (double)trajectory_length / trajectory_duration;
      boost::scoped_ptr<dmp_lib::Trajectory> trajectory(new dmp_lib::Trajectory());
      std::vector<std::string> variable_names;
      variable_names.push_back("ros_time");
      variable_names.insert(variable_names.end(), messages_[0].names.begin(), messages_[0].names.end());
      ROS_VERIFY(trajectory->initialize(variable_names, SAMPLING_FREQUENCY, true, trajectory_length));
      for (int i = 0; i < trajectory_length; ++i)
      {
        std::vector<double> data;
        data.push_back(static_cast<double>(messages_[i].header.stamp.toSec()));
        data.insert(data.end(), messages_[i].data.begin(), messages_[i].data.end());
        ROS_VERIFY(trajectory->add(data, true));
      }
      ROS_VERIFY(trajectory->writeToCLMCFile(file_name, true));
    }
    else
    {
      ROS_ERROR("Cannot write CLMC file when \"create_directories\" is disabled.");
      return false;
    }
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeResampledData()
  {
    return writeRecordedData(std::string("resampled"), true);
  }
template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRecordedDataSamples()
  {
    return writeRecordedData(std::string(""), true);
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRawData(const std::string raw_directory_name)
  {
    return writeRecordedData(raw_directory_name, false);
  }
template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRawData()
  {
    return writeRecordedData(std::string("raw"), false);
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeStatistics(std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    std::string file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(absolute_data_directory_path_) + task_recorder2_utilities::getStatFileName(prefixed_topic_name_, description_.trial);
    try
    {
      rosbag::Bag bag;
      bag.open(file_name, rosbag::bagmode::Write);
      for (int i = 0; i < static_cast<int> (vector_of_accumulated_trial_statistics.size()); ++i)
      {
        std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> accumulated_trial_statistics = vector_of_accumulated_trial_statistics[i];
        for (int j = 0; j < static_cast<int> (accumulated_trial_statistics.size()); ++j)
        {
          // accumulated_trial_statistics[j].id = getId(description_);
          bag.write(topic_name_, messages_[j].header.stamp, accumulated_trial_statistics[j]);
        }
      }
      bag.close();
    }
    catch (rosbag::BagIOException& ex)
    {
      ROS_ERROR("Problem when writing to bag file named >%s< : %s", file_name.c_str(), ex.what());
      return false;
    }
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::getList(std::vector<std::string>& descriptions)
  {
    boost::filesystem::path path = boost::filesystem::path(data_directory_name_);
    return task_recorder2_utilities::getDirectoryList(path, descriptions);
  }

}

#endif /* TASK_RECORDER_IO_H_ */
