/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    task_recorder.h

 \author  Peter Pastor, Mrinal Kalakrishnan
 \date    Jun 10, 2010

 *********************************************************************/

#ifndef TASK_RECORDER_H_
#define TASK_RECORDER_H_

// system includes
#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <filters/transfer_function.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/bspline.h>
#include <usc_utilities/logging.h>

// #include <task_recorder2_utilities/accumulator.h>
#include <task_recorder2_utilities/message_buffer.h>
#include <task_recorder2_utilities/message_ring_buffer.h>

#include <task_recorder2_utilities/data_sample_utilities.h>
#include <task_recorder2_utilities/task_description_utilities.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSample.h>

// local includes
#include <task_recorder2/task_recorder_io.h>
#include <task_recorder2/task_recorder_base.h>

// #include <task_recorder2_msgs/AccumulatedTrialStatistics.h>

namespace task_recorder2
{

const int MESSAGE_SUBSCRIBER_BUFFER_SIZE = 10000;

template<class MessageType>
  class TaskRecorder : public TaskRecorderBase
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    TaskRecorder(ros::NodeHandle node_handle);
    /*! Destructor
     */
    virtual ~TaskRecorder() {};

    /*!
     * @param topic_name
     * @param splining_method
     * @return True on success, otherwise False
     */
    bool initialize(const std::string topic_name,
                    const std::string splining_method = "BSpline");

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool startStreaming(task_recorder2::StartStreaming::Request& request,
                        task_recorder2::StartStreaming::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool startRecording(task_recorder2::StartRecording::Request& request,
                        task_recorder2::StartRecording::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool stopRecording(task_recorder2::StopRecording::Request& request,
                       task_recorder2::StopRecording::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool interruptRecording(task_recorder2::InterruptRecording::Request& request,
                            task_recorder2::InterruptRecording::Response& response);

    /*!
     * @param time
     * @param sample_data
     * @return True on success, otherwise False
     */
    bool getSampleData(const ros::Time& time,
                       task_recorder2_msgs::DataSample& data_sample);

    /*!
     * @return Topic name
     */
    std::string getTopicName() const
    {
      ROS_ASSERT_MSG(initialized_, "Task recorder is not initialized. Cannot return topic name.");
      return recorder_io_.topic_name_;
    }

  protected:

    /*!
     */
    bool initialized_;
    bool first_time_;

    /*!
     */
    TaskRecorderIO<task_recorder2_msgs::DataSample> recorder_io_;

    /*!
     * @param start_time
     * @param end_time
     * @param num_samples
     * @param message_names
     * @param filter_and_cropped_messages
     * @return True on success, otherwise False
     */
    bool filterAndCrop(const ros::Time& start_time,
                       const ros::Time& end_time,
                       const int num_samples,
                       const std::vector<std::string>& message_names,
                       std::vector<task_recorder2_msgs::DataSample>& filter_and_cropped_messages);
    /*!
     * @param start_time
     * @param end_time
     * @param num_samples
     * @param filter_and_cropped_messages
     * @return True on success, otherwise False
     */
    bool filterAndCrop(const ros::Time& start_time,
                       const ros::Time& end_time,
                       const int num_samples,
                       std::vector<task_recorder2_msgs::DataSample>& filter_and_cropped_messages);

    /*!
     * @param message
     * @param sample_data_msg
     * @return True on success, otherwise False
     */
    virtual bool transformMsg(const MessageType& message,
                              task_recorder2_msgs::DataSample& data_sample_msg) = 0;

    /*!
     * @return Number of signals
     */
    virtual int getNumSignals() const = 0;

    /*!
     * @param vector_of_accumulated_trial_statistics
     * @return True on success, otherwise False
     */
    // bool getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics);

    /*!
     */
    enum SpliningMethod
    {
      BSpline,    //!< BSpline
      Linear      //!< Linear
    };

  private:

    /*!
     */
    ros::ServiceServer start_recording_service_server_;
    ros::ServiceServer stop_recording_service_server_;

    /*!
     */
    ros::Subscriber message_subscriber_;

    /*!
     */
    ros::Time abs_start_time_;
    boost::mutex mutex_;
    bool logging_;
    bool streaming_;

    /*!
     */
    // task_recorder2_utilities::Accumulator accumulator_;
    boost::shared_ptr<task_recorder2_utilities::MessageRingBuffer> message_buffer_;
    // boost::shared_ptr<task_recorder2_utilities::MessageBuffer> message_buffer_;
    task_recorder2_msgs::DataSample data_sample_;

    /*!
     */
    int num_signals_;
    bool is_filtered_;
    filters::MultiChannelTransferFunctionFilter<double> filter_;
    std::vector<double> unfiltered_data_;
    std::vector<double> filtered_data_;

    /*!
     */
    SpliningMethod splining_method_;
    /*!
     * @param splining_method
     * @return True on success, otherwise False
     */
    bool setSpliningMethod(const std::string& splining_method);

    /*!
     */
    void startStreaming();

    /*!
     * @param description
     */
    void startRecording(const task_recorder2_msgs::Description& description);

    /*!
     */
    void stopRecording(bool stop_streaming);

    /*!
     * @param message
     */
    void recordMessagesCallback(const MessageTypeConstPtr message);

    /*!
     * @param messages
     * @param start_time
     * @param end_time
     * @param num_samples
     * @param message_names
     * @param resampled_messages
     * @return True on success, otherwise False
     */
    bool resample(std::vector<task_recorder2_msgs::DataSample>& messages,
                  const ros::Time& start_time,
                  const ros::Time& end_time,
                  const int num_samples,
                  const std::vector<std::string>& message_names,
                  std::vector<task_recorder2_msgs::DataSample>& resampled_messages);

    /*!
     * @param data_sample
     * @return True on success, otherwise False
     */
    bool filter(task_recorder2_msgs::DataSample& data_sample);

    /*!
     * @param index
     * @param input_vector
     * @param target_vector
     * @param input_querry
     * @param output_vector
     */
    void log(const int index,
             const std::vector<double>& input_vector,
             const std::vector<double>& target_vector,
             const std::vector<double>& input_querry,
             const std::vector<double>& output_vector);

    /*!
     */
    void waitForMessages();

    /*!
     * @return True if task_recorder2 is logging, False otherwise
     */
    inline bool isLogging();
    inline bool isStreaming();

};

template<class MessageType>
  TaskRecorder<MessageType>::TaskRecorder(ros::NodeHandle node_handle) :
    initialized_(false), first_time_(true), recorder_io_(node_handle), logging_(false), streaming_(false), /*accumulator_(node_handle),*/
    num_signals_(0), is_filtered_(false), splining_method_(BSpline)
  {
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::initialize(const std::string topic_name,
                                             const std::string splining_method)
  {
    if(!recorder_io_.initialize(topic_name))
    {
      ROS_ERROR("Could not initialize task recorder on topic >%s<.", topic_name.c_str());
      return (initialized_ = false);
    }
    ROS_VERIFY(setSpliningMethod(splining_method));

    std::string full_topic_name = topic_name;
    if(!task_recorder2_utilities::getTopicName(full_topic_name))
    {
      ROS_ERROR("Could not obtain topic name from name >%s<. Could not initialize base of the task recorder.", full_topic_name.c_str());
      return (initialized_ = false);
    }

    num_signals_ = getNumSignals();
    data_sample_.data.resize(num_signals_);
    data_sample_.names.resize(num_signals_);
    if(recorder_io_.node_handle_.hasParam(full_topic_name))
    {
      is_filtered_ = true;
      ROS_DEBUG("Filtering >%i< signals for >%s<.", num_signals_, full_topic_name.c_str());
      filtered_data_.resize(num_signals_, 0.0);
      unfiltered_data_.resize(num_signals_, 0.0);
      std::string parameter_name = recorder_io_.node_handle_.getNamespace() + std::string("/") + full_topic_name + std::string("/Filter");
      ROS_VERIFY(((filters::MultiChannelFilterBase<double>&)filter_).configure(num_signals_, parameter_name, recorder_io_.node_handle_));
    }

    start_recording_service_server_ = recorder_io_.node_handle_.advertiseService(std::string("start_recording_") + full_topic_name,
                                                                                 &TaskRecorder<MessageType>::startRecording, this);
    stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService(std::string("stop_recording_") + full_topic_name,
                                                                                &TaskRecorder<MessageType>::stopRecording, this);

    if(is_filtered_)
    {
      message_subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE, &TaskRecorder<MessageType>::recordMessagesCallback, this);
    }

    task_recorder2_msgs::DataSample default_data_sample;
    default_data_sample.names = getNames();
    default_data_sample.data.resize(default_data_sample.names.size(), 0.0);
    message_buffer_.reset(new task_recorder2_utilities::MessageRingBuffer(default_data_sample));
    // message_buffer_.reset(new task_recorder2_utilities::MessageBuffer());

    return (initialized_ = true);
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::setSpliningMethod(const std::string& splining_method)
  {
    if (splining_method.compare("BSpline") == 0)
    {
      splining_method_ = BSpline;
    }
    else if (splining_method.compare("Linear") == 0)
    {
      splining_method_ = Linear;
    }
    else
    {
      ROS_ERROR("Unknown splining method >%s<. Cannot initialize task recorder with topic >%s<.", splining_method.c_str(), splining_method.c_str());
      return false;
    }
    return true;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::recordMessagesCallback(const MessageTypeConstPtr message)
  {
    MessageType msg = *message;
    if(first_time_)
    {
      data_sample_.names = getNames();
    }
    ROS_VERIFY(transformMsg(msg, data_sample_));
    first_time_ = false;
    ROS_VERIFY(filter(data_sample_));
    mutex_.lock();
    if (logging_)
    {
      // double delay = (ros::Time::now() - data_sample_.header.stamp).toSec();
      // ROS_INFO("Delay = %f", delay);
      recorder_io_.messages_.push_back(data_sample_);
    }
    if (streaming_)
    {
      ROS_VERIFY(message_buffer_->add(data_sample_));
    }
    mutex_.unlock();
  }

template<class MessageType>
  void TaskRecorder<MessageType>::waitForMessages()
  {
    // wait for 1st message.
    bool no_message = true;
    while (no_message)
    {
      ros::spinOnce();
      mutex_.lock();
      no_message = recorder_io_.messages_.empty();
      if (!no_message)
      {
        abs_start_time_ = recorder_io_.messages_[0].header.stamp;
      }
      mutex_.unlock();
    }
  }

template<class MessageType>
  void TaskRecorder<MessageType>::startStreaming()
  {
    // TODO: send notification to task_monitor
    ROS_DEBUG("Start streaming topic named >%s<.", recorder_io_.topic_name_.c_str());
    if(!is_filtered_)
    {
      message_subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE, &TaskRecorder<MessageType>::recordMessagesCallback, this);
    }
    mutex_.lock();
    logging_ = false;
    streaming_ = true;
    mutex_.unlock();
  }

template<class MessageType>
  void TaskRecorder<MessageType>::startRecording(const task_recorder2_msgs::Description& description)
  {
    // TODO: send notification to task_monitor
    ROS_DEBUG("Start recording topic named >%s< with description >%s< to id >%i<.",
        recorder_io_.topic_name_.c_str(), task_recorder2_utilities::getDescription(description).c_str(), task_recorder2_utilities::getId(description));
    recorder_io_.setResampledDescription(description);
    if(!is_filtered_)
    {
      message_subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE, &TaskRecorder<MessageType>::recordMessagesCallback, this);
    }
    mutex_.lock();
    logging_ = true;
    streaming_ = true;
    recorder_io_.messages_.clear();
    mutex_.unlock();
    waitForMessages();
  }

template<class MessageType>
  void TaskRecorder<MessageType>::stopRecording(bool stop_streaming)
  {
    ROS_DEBUG("Stop recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    mutex_.lock();
    // ROS_ASSERT_MSG(logging_, "Task recorder is currently not recording... not stopping.");
    if(logging_)
    {
      logging_ = false;
    }
    streaming_ = stop_streaming;
    mutex_.unlock();
    if(!is_filtered_)
    {
      message_subscriber_.shutdown();
    }
    // TODO: ask for label
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::startStreaming(task_recorder2::StartStreaming::Request& request,
                                                 task_recorder2::StartStreaming::Response& response)
  {
    startStreaming();
    response.return_code = task_recorder2::StartStreaming::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::startRecording(task_recorder2::StartRecording::Request& request,
                                                 task_recorder2::StartRecording::Response& response)
  {
    startRecording(request.description);
    response.start_time = abs_start_time_;
    response.return_code = task_recorder2::StartRecording::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::stopRecording(task_recorder2::StopRecording::Request& request,
                                                task_recorder2::StopRecording::Response& response)
  {
    stopRecording(request.stop_streaming);

    if (recorder_io_.write_out_resampled_data_ || recorder_io_.write_out_statistics_)
    {
      if (!filterAndCrop(request.crop_start_time, request.crop_end_time, request.num_samples, request.message_names, response.filtered_and_cropped_messages))
      {
        response.info = std::string("Could not filter and crop messages of topic >" + recorder_io_.topic_name_ + "<. ");
        ROS_ERROR_STREAM(response.info);
        response.return_code = task_recorder2::StopRecording::Response::SERVICE_CALL_FAILED;
        return true;
      }
    }

    if (recorder_io_.write_out_resampled_data_)
    {
      boost::thread(boost::bind(&TaskRecorderIO<task_recorder2_msgs::DataSample>::writeResampledData, recorder_io_));
    }

    // if(recorder_io_.write_out_statistics_)
    // {
    //   std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> > vector_of_accumulated_trial_statistics;
    //   ROS_VERIFY(getAccumulatedTrialStatistics(vector_of_accumulated_trial_statistics));
    //   boost::thread(boost::bind(&task_recorder2_utilities::TaskRecorderIO<task_recorder2_msgs::DataSample>::writeStatistics, recorder_io_, vector_of_accumulated_trial_statistics));
    // }

    ROS_DEBUG("Stopped recording topic >%s< and returning >%i< messages.",
              recorder_io_.topic_name_.c_str(), (int)response.filtered_and_cropped_messages.size());
    response.description = recorder_io_.getDescription();
    response.info = std::string("Stopped recording >" + recorder_io_.topic_name_ + "<. ");
    response.return_code = task_recorder2::StopRecording::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::interruptRecording(task_recorder2::InterruptRecording::Request& request,
                                                     task_recorder2::InterruptRecording::Response& response)
  {
    stopRecording(false);
    ROS_DEBUG("Interrupted recording topic >%s<.", recorder_io_.topic_name_.c_str());
    response.info = std::string("Stopped recording >" + recorder_io_.topic_name_ + "<. ");
    response.return_code = task_recorder2::InterruptRecording::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::filterAndCrop(const ros::Time& start_time,
                                                const ros::Time& end_time,
                                                const int num_samples,
                                                std::vector<task_recorder2_msgs::DataSample>& filter_and_cropped_messages)
  {
    std::vector<std::string> no_message_names;
    return filterAndCrop(start_time, end_time, num_samples, no_message_names, filter_and_cropped_messages);
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::filterAndCrop(const ros::Time& start_time,
                                                const ros::Time& end_time,
                                                const int num_samples,
                                                const std::vector<std::string>& message_names,
                                                std::vector<task_recorder2_msgs::DataSample>& filter_and_cropped_messages)
  {
    int num_messages = (int)recorder_io_.messages_.size();
    if (num_messages == 0)
    {
      ROS_ERROR("Zero messages have been logged.");
      return false;
    }

    filter_and_cropped_messages.clear();
    if(num_samples == 1) // only 1 sample requested
    {
      task_recorder2_msgs::DataSample data_sample;
      ROS_VERIFY(message_buffer_->get(start_time, data_sample));
      data_sample.header.stamp = ros::TIME_MIN;
      filter_and_cropped_messages.push_back(data_sample);
      if(recorder_io_.write_out_raw_data_)
      {
        boost::thread(boost::bind(&TaskRecorderIO<task_recorder2_msgs::DataSample>::writeRawData, recorder_io_));
      }
      recorder_io_.messages_ = filter_and_cropped_messages;
      return true;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
    ros::Time our_end_time = recorder_io_.messages_[num_messages - 1].header.stamp;

    int index = 0;
    while (our_end_time.toSec() < 1e-6)
    {
      index++;
      if((1+index) > num_messages)
      {
        ROS_ERROR("Time stamps of recorded messages seem to be invalid.");
        return false;
      }
      our_end_time = recorder_io_.messages_[num_messages - (1 + index)].header.stamp;
    }

    if (our_start_time > start_time || our_end_time < end_time)
    {
      ROS_ERROR("Requested times have not been recorded!");
      ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
      ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
      return false;
    }

    // first crop
    ROS_VERIFY(task_recorder2_utilities::crop<task_recorder2_msgs::DataSample>(recorder_io_.messages_, start_time, end_time));
    // then remove duplicates
    ROS_VERIFY(task_recorder2_utilities::removeDuplicates<task_recorder2_msgs::DataSample>(recorder_io_.messages_));

    if(recorder_io_.write_out_raw_data_)
    {
      boost::thread(boost::bind(&TaskRecorderIO<task_recorder2_msgs::DataSample>::writeRawData, recorder_io_));
    }

    // then resample
    ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples, message_names, filter_and_cropped_messages));
    ROS_ASSERT(static_cast<int>(filter_and_cropped_messages.size()) == num_samples);

    recorder_io_.messages_ = filter_and_cropped_messages;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::resample(std::vector<task_recorder2_msgs::DataSample>& messages,
                                           const ros::Time& start_time,
                                           const ros::Time& end_time,
                                           const int num_samples,
                                           const std::vector<std::string>& message_names,
                                           std::vector<task_recorder2_msgs::DataSample>& resampled_messages)
  {
    // error checking
    ROS_ASSERT(!messages.empty());
    for (int i = 0; i < (int)messages.size(); ++i)
    {
      ROS_ASSERT(!messages[i].names.empty());
      ROS_ASSERT(messages[i].names.size() == messages[i].data.size());
    }
    ROS_ASSERT(num_samples > 1);

    // extract indices
    std::vector<std::string> selected_names = message_names;
    if(selected_names.empty())
    {
      selected_names = messages[0].names;
    }

    std::vector<int> indices;
    ROS_VERIFY(task_recorder2_utilities::getIndices(messages[0].names, selected_names, indices));

    const int num_messages = static_cast<int> (messages.size());
    const int num_vars = static_cast<int>(indices.size());

    // compute mean dt of the provided time stamps
    double dts[num_messages - 1];
    double mean_dt = 0.0;

    std::vector<double> input_vector(num_messages);
    input_vector[0] = messages[0].header.stamp.toSec();
    for (int i = 0; i < num_messages - 1; i++)
    {
      dts[i] = messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec();
      mean_dt += dts[i];
      input_vector[i + 1] = input_vector[i] + dts[i];
    }
    mean_dt /= static_cast<double> (num_messages - 1);

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));
    double wave_length = interval.toSec() * static_cast<double> (2.0);

    resampled_messages.clear();
    std::vector<double> input_querry(num_samples);
    for (int i = 0; i < num_samples; i++)
    {
      task_recorder2_msgs::DataSample msg;
      msg.header.seq = i;
      msg.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
      msg.header.frame_id = messages[0].header.frame_id;
      input_querry[i] = msg.header.stamp.toSec();
      resampled_messages.push_back(msg);
    }

    std::vector<std::vector<double> > variables;
    variables.resize(num_vars);
    for (int j = 0; j < num_messages; ++j)
    {
      for (int i = 0; i < num_vars; ++i)
      {
        variables[i].push_back(messages[j].data[indices[i]]);
      }
    }

    std::vector<std::vector<double> > variables_resampled;
    variables_resampled.resize(num_vars);
    for (int i = 0; i < num_vars; ++i)
    {
      switch(splining_method_)
      {
        case BSpline:
        {
          ROS_VERIFY(usc_utilities::resample(input_vector, variables[i], wave_length, input_querry, variables_resampled[i], false));
          // log(i, input_vector, variables[i], input_querry, variables_resampled[i]);
          break;
        }
        case Linear:
        {
          ROS_VERIFY(usc_utilities::resampleLinearNoBounds(input_vector, variables[i], input_querry, variables_resampled[i]));
          // log(i, input_vector, variables[i], input_querry, variables_resampled[i]);
          break;
        }
        default:
        {
          ROS_ASSERT_MSG(false, "Unknown sampling method for task recorder with topic >%s<. This should never happen.", recorder_io_.topic_name_.c_str());
        }
      }
    }

    std::vector<std::string> names;
    for (int i = 0; i < num_vars; ++i)
    {
      names.push_back(messages[0].names[indices[i]]);
    }
    for (int j = 0; j < num_samples; ++j)
    {
      resampled_messages[j].data.resize(num_vars, 0.0);
      for (int i = 0; i < num_vars; ++i)
      {
        resampled_messages[j].data[i] = variables_resampled[i][j];
      }
      resampled_messages[j].names = names;
      // make time stamps start from 0.0
      resampled_messages[j].header.stamp = static_cast<ros::Time> (ros::TIME_MIN + ros::Duration(j * interval.toSec()));
    }
    return true;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::log(const int index,
                                      const std::vector<double>& input_vector,
                                      const std::vector<double>& target_vector,
                                      const std::vector<double>& input_querry,
                                      const std::vector<double>& output_vector)
  {
    if (index == 0)
    {
      usc_utilities::log(input_vector, "/tmp/input.txt");
      usc_utilities::log(input_querry, "/tmp/querry.txt");
    }
    std::stringstream ss;
    ss << index;
    usc_utilities::log(target_vector, std::string(std::string("/tmp/target" + ss.str() + ".txt")).c_str());
    usc_utilities::log(output_vector, std::string(std::string("/tmp/resampled" + ss.str() + ".txt")).c_str());
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::filter(task_recorder2_msgs::DataSample& data_sample)
  {
    // skip filtering if not filter has been specified
    if(!is_filtered_)
    {
      return true;
    }
    unfiltered_data_ = data_sample.data;
    ROS_VERIFY(filter_.update(unfiltered_data_, filtered_data_));
    data_sample.data = filtered_data_;
    return true;
  }

//template<class MessageType>
//  bool TaskRecorder<MessageType>::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
//  {
////    vector_of_accumulated_trial_statistics.clear();
////    std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> accumulated_trial_statistics;
////    ROS_VERIFY(accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
////    setMessageNames(accumulated_trial_statistics);
////    vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
//    return true;
//  }

template<class MessageType>
  bool TaskRecorder<MessageType>::getSampleData(const ros::Time& time, task_recorder2_msgs::DataSample& data_sample)
  {
    if(isStreaming())
    {
      return message_buffer_->get(time, data_sample);
    }
    return false;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::isLogging()
  {
    bool is_logging;
    mutex_.lock();
    is_logging = logging_;
    mutex_.unlock();
    return is_logging;
  }
template<class MessageType>
  bool TaskRecorder<MessageType>::isStreaming()
  {
    bool is_streaming;
    mutex_.lock();
    is_streaming = streaming_;
    mutex_.unlock();
    return is_streaming;
  }

}

#endif /* TASK_RECORDER_H_ */
