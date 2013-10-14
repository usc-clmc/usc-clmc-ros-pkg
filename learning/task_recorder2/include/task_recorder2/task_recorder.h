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

#include <task_recorder2_io/task_recorder_io.h>

// local includes
#include <task_recorder2/task_recorder_base.h>

// #include <task_recorder2_msgs/AccumulatedTrialStatistics.h>

namespace task_recorder2
{

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
     * @param service_prefix
     * @param splining_method
     * @param variable_name_prefix
     * @return True on success, otherwise False
     */
    bool initialize(const std::string topic_name,
                    const std::string service_prefix = "",
                    const std::string splining_method = "Linear",
                    const std::string variable_name_prefix = "")
    {
      task_recorder2_msgs::TaskRecorderSpecification task_recorder_specification;
      task_recorder_specification.topic_name = topic_name;
      task_recorder_specification.service_prefix = service_prefix;
      task_recorder_specification.variable_name_prefix = variable_name_prefix;
      task_recorder_specification.splining_method = splining_method;
      return initialize(task_recorder_specification);
    }

    /*!
     * @param task_recorder_specification
     * @return True on success, otherwise False
     */
    bool initialize(const task_recorder2_msgs::TaskRecorderSpecification& task_recorder_specification);

    /*!
     * @param node_handle The node_handle that is specific to the (particular) task recorder
     * Derived classes can implement this function to retrieve (arm) specific parameters
     * @return True on success, otherwise False
     */
    bool readParams(ros::NodeHandle& node_handle)
    {
      return true;
    }
    bool readParams(ros::NodeHandle& node_handle, const std::string& class_name_prefix)
    {
      variable_name_prefix_ = "";
      if (!class_name_prefix.empty())
      {
        ros::NodeHandle private_node_handle("/TaskRecorderManager/" + class_name_prefix);
        if(private_node_handle.hasParam("variable_name_prefix"))
        {
          ROS_VERIFY(usc_utilities::read(private_node_handle, "variable_name_prefix", variable_name_prefix_));
        }
      }
      return readParams(node_handle);
    }

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool startStreaming(task_recorder2_srvs::StartStreaming::Request& request,
                        task_recorder2_srvs::StartStreaming::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool stopStreaming(task_recorder2_srvs::StopStreaming::Request& request,
                       task_recorder2_srvs::StopStreaming::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool startRecording(task_recorder2_srvs::StartRecording::Request& request,
                        task_recorder2_srvs::StartRecording::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool stopRecording(task_recorder2_srvs::StopRecording::Request& request,
                       task_recorder2_srvs::StopRecording::Response& response);

    /*!
     * @param request
     * @param response
     * @return True on success, otherwise False
     */
    bool interruptRecording(task_recorder2_srvs::InterruptRecording::Request& request,
                            task_recorder2_srvs::InterruptRecording::Response& response);

    /*!
     * @param first
     * @param last
     * @return True if successful (i.e. recorder is currently recording), otherwise False
     */
    bool getTimeStamps(ros::Time& first, ros::Time& last);

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
    task_recorder2_io::TaskRecorderIO<task_recorder2_msgs::DataSample> recorder_io_;

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
     * This function will be called right before each recording is started
     * It allowes derived classes to initialize before each recording
     * @return True on success, otherwise False
     */
    bool startRecording()
    {
      return true;
    }

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

    /*! Variable name prefix, default is empty.
     */
    std::string variable_name_prefix_;

    /*!
     */
    static const int MESSAGE_SUBSCRIBER_BUFFER_SIZE = 10000;

    /*!
     */
    ros::ServiceServer start_recording_service_server_;
    ros::ServiceServer stop_recording_service_server_;

    /*!
     */
    ros::Subscriber message_subscriber_;

    /*!
     */
    ros::Timer message_timer_;

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
     * @param description
     * @return True on success, otherwise False
     */
    bool startRecording(const task_recorder2_msgs::Description& description);

    /*!
     * @param message
     */
    void recordMessagesCallback(const MessageTypeConstPtr message);

    /*!
     */
    void recordTimerCallback(const ros::TimerEvent& timer_event);

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
     * @param name
     * @param index
     * @param input_vector
     * @param target_vector
     * @param input_querry
     * @param output_vector
     */
    void log(const std::string& name,
             const int index,
             const std::vector<double>& input_vector,
             const std::vector<double>& target_vector,
             const std::vector<double>& input_querry,
             const std::vector<double>& output_vector);

    /*!
     */
    void processDataSample(const MessageType& msg);

    /*!
     */
    void waitForMessages();

    /*!
     */
    inline void setLogging(bool logging);
    inline void setStreaming(bool streaming);

    /*!
     * @return True if recorder is streaming, False otherwise
     */
    inline bool isStreaming();

    /*!
     * @return True if recorder is currently recording, otherwise False
     */
    inline bool isRecording();

    /*! Adds the variable prefix to all variable names
     * @param names
     */
    void addVariablePrefix(std::vector<std::string>& names)
    {
      for (unsigned int i = 0; i < names.size(); ++i)
      {
        names[i] = variable_name_prefix_ + names[i];
      }
    }

};

template<class MessageType>
  TaskRecorder<MessageType>::TaskRecorder(ros::NodeHandle node_handle) :
    initialized_(false), first_time_(true), recorder_io_(ros::NodeHandle("/TaskRecorderManager")), variable_name_prefix_(""),
    logging_(false), streaming_(false), /*accumulator_(node_handle),*/
    num_signals_(0), is_filtered_(false), splining_method_(BSpline)
  {
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::initialize(const task_recorder2_msgs::TaskRecorderSpecification& task_recorder_specification)
  {
    if(!recorder_io_.initialize(task_recorder_specification.topic_name, task_recorder_specification.service_prefix))
    {
      ROS_ERROR("Could not initialize task recorder on topic >%s< with prefix >%s<.",
                task_recorder_specification.topic_name.c_str(), task_recorder_specification.service_prefix.c_str());
      return (initialized_ = false);
    }
    ROS_VERIFY(setSpliningMethod(task_recorder_specification.splining_method));

    std::string full_topic_name = task_recorder_specification.topic_name;
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

    start_recording_service_server_ = recorder_io_.node_handle_.advertiseService(std::string("start_recording_") + task_recorder_specification.service_prefix + full_topic_name,
                                                                                 &TaskRecorder<MessageType>::startRecording, this);
    stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService(std::string("stop_recording_") + task_recorder_specification.service_prefix + full_topic_name,
                                                                                &TaskRecorder<MessageType>::stopRecording, this);

    if (task_recorder_specification.message_timer_rate < 0.0) // record messages by subscribing to the topic
    {
      // if(!is_filtered_)
      // {
      message_subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE, &TaskRecorder<MessageType>::recordMessagesCallback, this);
      // }
    }
    else // record messages at a fixed rate (polling)
    {
      message_timer_ = recorder_io_.node_handle_.createTimer(ros::Duration(1.0/task_recorder_specification.message_timer_rate), &TaskRecorder<MessageType>::recordTimerCallback, this);
      // TODO: Read this from param server
      data_sample_.header.frame_id.assign("/BASE");
    }

    task_recorder2_msgs::DataSample default_data_sample;
    default_data_sample.names = getNames();
    addVariablePrefix(default_data_sample.names);
    // write variable names onto param server
    ros::NodeHandle private_node_handle(recorder_io_.node_handle_, task_recorder_specification.class_name);
    ROS_VERIFY(usc_utilities::write(private_node_handle, "variable_names", default_data_sample.names));
    default_data_sample.data.resize(default_data_sample.names.size(), 0.0);
    message_buffer_.reset(new task_recorder2_utilities::MessageRingBuffer(default_data_sample));
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
    // ROS_INFO("Message callback for topic >%s<.", recorder_io_.topic_name_.c_str());
    processDataSample(*message);
  }

template<class MessageType>
  void TaskRecorder<MessageType>::recordTimerCallback(const ros::TimerEvent& timer_event)
  {
    // ROS_INFO("Timer callback for topic >%s<.", recorder_io_.topic_name_.c_str());
    MessageType empty_msg;
    // TODO: think about which timer event should be used
    data_sample_.header.stamp = timer_event.current_expected;
    processDataSample(empty_msg);
  }

template<class MessageType>
  void TaskRecorder<MessageType>::processDataSample(const MessageType& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(logging_ || streaming_)
    {
      if(!transformMsg(msg, data_sample_))
      {
        return;
      }
      if(first_time_)
      {
        data_sample_.names = getNames();
        addVariablePrefix(data_sample_.names);
        first_time_ = false;
      }
      ROS_VERIFY(filter(data_sample_));
    }
    if (logging_)
    {
      // double delay = (ros::Time::now() - data_sample_.header.stamp).toSec();
      // ROS_INFO("Delay = %f", delay);
      // ROS_INFO("Logging >%s<.", recorder_io_.topic_name_.c_str());
      recorder_io_.messages_.push_back(data_sample_);
    }
    if (streaming_)
    {
      // ROS_INFO("Streaming >%s<.", recorder_io_.topic_name_.c_str());
      ROS_VERIFY(message_buffer_->add(data_sample_));
    }
  }

template<class MessageType>
  void TaskRecorder<MessageType>::waitForMessages()
  {
    // wait for 1st message.
    bool no_message = true;
    ROS_DEBUG("Waiting for message >%s<", recorder_io_.topic_name_.c_str());
    int counter = 0;
    while (ros::ok() && no_message)
    {
      ros::spinOnce();
      mutex_.lock();
      no_message = recorder_io_.messages_.empty();
      if (!no_message)
      {
        abs_start_time_ = recorder_io_.messages_[0].header.stamp;
      }
      mutex_.unlock();
      ros::Duration(0.01).sleep();
      counter++;
      if (counter > 100)
      {
        ROS_WARN("Waiting for message on topic >%s<.", recorder_io_.topic_name_.c_str());
        counter = 0;
      }
    }
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::startRecording(const task_recorder2_msgs::Description& description)
  {
    // TODO: send notification to task_monitor
    ROS_DEBUG("Start recording topic named >%s< with description >%s< to id >%i<.",
      recorder_io_.topic_name_.c_str(), task_recorder2_utilities::getDescription(description).c_str(), task_recorder2_utilities::getId(description));
    recorder_io_.setResampledDescription(description);
    // if(!is_filtered_)
    // {
    // message_subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE, &TaskRecorder<MessageType>::recordMessagesCallback, this);
    // }
    mutex_.lock();
    logging_ = true;
    recorder_io_.messages_.clear();
    mutex_.unlock();
    if (!startRecording())
    {
      ROS_ERROR("Problem starting to record >%s< on topic >%s<.",
                task_recorder2_utilities::getDescription(description).c_str(), recorder_io_.topic_name_.c_str());
      return false;
    }
    waitForMessages();
    ROS_DEBUG("Recording topic named >%s< with description >%s< to id >%i<.",
        recorder_io_.topic_name_.c_str(), task_recorder2_utilities::getDescription(description).c_str(), task_recorder2_utilities::getId(description));
    return true;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::setLogging(bool logging)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG_COND(logging_ && !logging, "Stop recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    ROS_DEBUG_COND(!logging_ && logging, "Start recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    logging_ = logging;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::setStreaming(bool streaming)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG_COND(streaming_ && !streaming, "Stop streaming topic named >%s<.", recorder_io_.topic_name_.c_str());
    ROS_DEBUG_COND(!streaming_ && streaming, "Start streaming topic named >%s<.", recorder_io_.topic_name_.c_str());
    streaming_ = streaming;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::getTimeStamps(ros::Time& first, ros::Time& last)
  {
    if(!isRecording())
    {
      return false;
    }
    first = abs_start_time_;
    mutex_.lock();
    if (logging_)
    {
      last = recorder_io_.messages_.back().header.stamp;
    }
    mutex_.unlock();
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::startStreaming(task_recorder2_srvs::StartStreaming::Request& request,
                                                 task_recorder2_srvs::StartStreaming::Response& response)
  {
    setStreaming(true);
    response.info = std::string("Started streaming >" + recorder_io_.topic_name_ + "<. ");
    response.return_code = task_recorder2_srvs::StartStreaming::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::stopStreaming(task_recorder2_srvs::StopStreaming::Request& request,
                                                task_recorder2_srvs::StopStreaming::Response& response)
  {
    setStreaming(false);
    response.info = std::string("Stopped streaming >" + recorder_io_.topic_name_ + "<. ");
    response.return_code = task_recorder2_srvs::StopStreaming::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }


template<class MessageType>
  bool TaskRecorder<MessageType>::startRecording(task_recorder2_srvs::StartRecording::Request& request,
                                                 task_recorder2_srvs::StartRecording::Response& response)
  {
    if(!startRecording(request.description))
    {
      response.return_code = task_recorder2_srvs::StartRecording::Response::SERVICE_CALL_FAILED;
      return true;
    }
    response.start_time = abs_start_time_;
    response.return_code = task_recorder2_srvs::StartRecording::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::stopRecording(task_recorder2_srvs::StopRecording::Request& request,
                                                task_recorder2_srvs::StopRecording::Response& response)
  {
    setLogging(!request.stop_recording);
    if (!filterAndCrop(request.crop_start_time, request.crop_end_time, request.num_samples, request.message_names,
                       response.filtered_and_cropped_messages))
    {
      response.info = std::string("Could not filter and crop messages of topic >" + recorder_io_.topic_name_ + "<. ");
      ROS_ERROR_STREAM(response.info);
      response.return_code = task_recorder2_srvs::StopRecording::Response::SERVICE_CALL_FAILED;
      return true;
    }

    if (recorder_io_.write_out_resampled_data_)
    {
      boost::thread(boost::bind(&task_recorder2_io::TaskRecorderIO<task_recorder2_msgs::DataSample>::writeResampledData, recorder_io_));
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
    response.return_code = task_recorder2_srvs::StopRecording::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::interruptRecording(task_recorder2_srvs::InterruptRecording::Request& request,
                                                     task_recorder2_srvs::InterruptRecording::Response& response)
  {
    setLogging(false);
    ROS_DEBUG("Interrupted recording topic >%s<.", recorder_io_.topic_name_.c_str());
    response.info = std::string("Stopped recording >" + recorder_io_.topic_name_ + "<. ");
    response.return_code = task_recorder2_srvs::InterruptRecording::Response::SERVICE_CALL_SUCCESSFUL;
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
        boost::thread(boost::bind(&task_recorder2_io::TaskRecorderIO<task_recorder2_msgs::DataSample>::writeRawData, recorder_io_));
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
      boost::thread(boost::bind(&task_recorder2_io::TaskRecorderIO<task_recorder2_msgs::DataSample>::writeRawData, recorder_io_));
    }

    ROS_DEBUG("Resampling >%i< messages to >%i< messages for topic >%s<.", (int)recorder_io_.messages_.size(), num_samples, recorder_io_.topic_name_.c_str());

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
    ros::Time first_time_stamp = messages[0].header.stamp;
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
          // log(task_recorder2_utilities::getDataFileName(recorder_io_.prefixed_topic_name_, i), i, input_vector, variables[i], input_querry, variables_resampled[i]);
          break;
        }
        case Linear:
        {
          ROS_VERIFY(usc_utilities::resampleLinearNoBounds(input_vector, variables[i], input_querry, variables_resampled[i]));
          // log(task_recorder2_utilities::getDataFileName(recorder_io_.prefixed_topic_name_, i), i, input_vector, variables[i], input_querry, variables_resampled[i]);
          break;
        }
        default:
        {
          ROS_ASSERT_MSG(false, "Unknown sampling method for task recorder with topic >%s<. This should never happen.", recorder_io_.topic_name_.c_str());
          break;
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
      // resampled_messages[j].header.stamp = static_cast<ros::Time> (ros::TIME_MIN + ros::Duration(j * interval.toSec()));
      resampled_messages[j].header.stamp = static_cast<ros::Time> (first_time_stamp - ros::Duration(task_recorder2_io::ROS_TIME_OFFSET) + ros::Duration(j * interval.toSec()));
    }
    return true;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::log(const std::string& name,
                                      const int index,
                                      const std::vector<double>& input_vector,
                                      const std::vector<double>& target_vector,
                                      const std::vector<double>& input_querry,
                                      const std::vector<double>& output_vector)
  {
    if (index == 0)
    {
      usc_utilities::log(input_vector, "/tmp/input_" + name + ".txt");
      usc_utilities::log(input_querry, "/tmp/querry_" + name + ".txt");
    }
    std::stringstream ss;
    ss << index;
    usc_utilities::log(target_vector, std::string(std::string("/tmp/target_" + name + "_" + ss.str() + ".txt")).c_str());
    usc_utilities::log(output_vector, std::string(std::string("/tmp/resampled_" + name + "_" + ss.str() + ".txt")).c_str());
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::filter(task_recorder2_msgs::DataSample& data_sample)
  {
    // skip filtering if no filter has been specified
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
  bool TaskRecorder<MessageType>::isRecording()
  {
    bool is_logging;
    boost::mutex::scoped_lock lock(mutex_);
    is_logging = logging_;
    return is_logging;
  }
template<class MessageType>
  bool TaskRecorder<MessageType>::isStreaming()
  {
    bool is_streaming;
    boost::mutex::scoped_lock lock(mutex_);
    is_streaming = streaming_;
    return is_streaming;
  }

}

#endif /* TASK_RECORDER_H_ */
