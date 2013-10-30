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
// #include <ros/callback_queue.h>
#include <filters/transfer_function.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/bspline.h>
#include <usc_utilities/logging.h>

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

namespace task_recorder2
{

template<class MessageType>
  class TaskRecorder : public TaskRecorderBase
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    TaskRecorder();
    /*! Destructor
     */
    virtual ~TaskRecorder();

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
     * @param is_recording
     * @param is_streaming
     * @param num_recorded_messages
     * @return True if at least one sample has been recorded, meaning that
     * the first/last time stamps are actually meaningful, otherwise False
     */
    bool getTimeStamps(ros::Time& first,
                       ros::Time& last,
                       bool& is_recording,
                       bool& is_streaming,
                       unsigned int& num_recorded_messages);
    /*!
     * @param first
     * @param last
     * @return True if at least one sample has been recorded, meaning that
     * the first/last time stamps are actually meaningful, otherwise False
     */
    bool getTimeStamps(ros::Time& first, ros::Time& last)
    {
      bool is_recording = false;
      bool is_streaming = false;
      unsigned int num_recorded_messages = 0;
      return getTimeStamps(first, last, is_recording, is_streaming, num_recorded_messages);
    }

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
      return recorder_io_.topic_name_;
    }

  protected:

    /*!
     */
    bool first_time_;

    /*!
     */
    task_recorder2_io::TaskRecorderIO<task_recorder2_msgs::DataSample> recorder_io_;

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
     * It allows derived classes to initialize before each recording
     * @return True on success, otherwise False
     */
    bool startRecording()
    {
      return true;
    }

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
    static const unsigned int MESSAGE_SUBSCRIBER_BUFFER_SIZE = 10000;

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
    // ros::NodeHandle callback_node_handle_;
    // ros::CallbackQueue callback_queue_;
    // boost::shared_ptr<ros::AsyncSpinner> async_spinner_;

    /*!
     */
    ros::Time abs_start_time_;
    boost::mutex mutex_;
    bool is_recording_;
    bool is_streaming_;

    /*! Used for streaming. The buffer never gets deleted so no mutex required
     */
    boost::shared_ptr<task_recorder2_utilities::MessageRingBuffer> message_buffer_;

    /*!
     */
    task_recorder2_msgs::DataSample data_sample_;

    /*!
     */
    int num_signals_;
    bool is_filtered_;
    filters::MultiChannelTransferFunctionFilter<double> filter_;
    std::vector<double> unfiltered_data_;
    std::vector<double> filtered_data_;

    /*! Check whether variable names do not exceed maximum length (required by CLMC plot and friends)
     * @param variable_names
     * @return True on success, otherwise False
     */
    bool areVariableNamesValid(const std::vector<std::string>& variable_names,
                                        const unsigned int maximum_length = 20);

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
                  const unsigned int num_samples,
                  const std::vector<std::string>& message_names,
                  std::vector<task_recorder2_msgs::DataSample>& resampled_messages);

    //    /*!
    //     * @param data_sample
    //     * @return True on success, otherwise False
    //     */
    //    bool filter(task_recorder2_msgs::DataSample& data_sample);

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
    void waitForMessage(const unsigned int num_messages_already_received,
                        ros::Time& stamp,
                        const bool update_abs_start_time);
    void waitForMessage(const unsigned int num_messages_already_received = 0,
                        const bool update_abs_start_time = false)
    {
      ros::Time stamp;
      waitForMessage(num_messages_already_received, stamp, update_abs_start_time);
    }

    /*! Turns recording On/Off
     * @param recording
     */
    inline void setRecording(const bool recording);
    /*! Turns streaming On/Off
     * @param streaming
     */
    inline void setStreaming(const bool streaming);

    /*!
     * @return True if recorder is currently recording, otherwise False
     */
    inline bool isRecording();
    /*!
     * @param num_messages
     * @return True if recorder is currently recording, otherwise False
     */
    inline bool isRecording(unsigned int& num_messages);
    /*!
     * @return True if recorder is streaming, False otherwise
     */
    inline bool isStreaming();

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

    /*!
     * @param start_time
     * @param end_time
     * @param interrupt_start_stamps
     * @param interrupt_durations
     * @param num_samples
     * @param message_names
     * @return True on success, otherwise False
     */
    bool filterAndCrop(const ros::Time& start_time,
                       const ros::Time& end_time,
                       const std::vector<ros::Time>& interrupt_start_stamps,
                       const std::vector<ros::Duration>& interrupt_durations,
                       const unsigned int num_samples,
                       const std::vector<std::string>& message_names);
    bool filterAndCrop(const ros::Time& start_time,
                       const ros::Time& end_time,
                       const std::vector<ros::Time>& interrupt_start_stamps,
                       const std::vector<ros::Duration>& interrupt_durations,
                       const unsigned int num_samples)
    {
      std::vector<std::string> no_message_names;
      return filterAndCrop(start_time, end_time, interrupt_start_stamps, interrupt_durations, num_samples, no_message_names);
    }

    /*!
     * @param info
     * @param response
     * @return True on success, otherwise False
     */
    bool fail(const std::string& info,
              task_recorder2_srvs::StopRecording::Response& response);

};

template<class MessageType>
  TaskRecorder<MessageType>::TaskRecorder() :
    first_time_(true), recorder_io_(ros::NodeHandle("/TaskRecorderManager")), variable_name_prefix_(""),
    is_recording_(false), is_streaming_(false),
    num_signals_(0), is_filtered_(false), splining_method_(Linear)
  {
  }

template<class MessageType>
  TaskRecorder<MessageType>::~TaskRecorder()
  {
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::initialize(const task_recorder2_msgs::TaskRecorderSpecification& task_recorder_specification)
  {
    if (!recorder_io_.initialize(task_recorder_specification.topic_name, task_recorder_specification.service_prefix))
    {
      ROS_ERROR("Could not initialize task recorder on topic >%s< with prefix >%s<.",
                task_recorder_specification.topic_name.c_str(), task_recorder_specification.service_prefix.c_str());
      return false;
    }
    ROS_VERIFY(setSpliningMethod(task_recorder_specification.splining_method));

    std::string full_topic_name = task_recorder_specification.topic_name;
    if (!task_recorder2_utilities::getTopicName(full_topic_name))
    {
      ROS_ERROR("Could not obtain topic name from name >%s<. Could not initialize base of the task recorder.", full_topic_name.c_str());
      return false;
    }

    num_signals_ = getNumSignals();
    data_sample_.data.resize(num_signals_);
    data_sample_.names.resize(num_signals_);
    if (recorder_io_.node_handle_.hasParam(full_topic_name))
    {
      is_filtered_ = true;
      ROS_DEBUG("Filtering >%i< signals for >%s<.", num_signals_, full_topic_name.c_str());
      filtered_data_.resize(num_signals_, 0.0);
      unfiltered_data_.resize(num_signals_, 0.0);
      std::string parameter_name = recorder_io_.node_handle_.getNamespace() + "/" + full_topic_name + "/Filter";
      ROS_VERIFY(((filters::MultiChannelFilterBase<double>&)filter_).configure(num_signals_, parameter_name, recorder_io_.node_handle_));
    }

    task_recorder2_msgs::DataSample default_data_sample;
    default_data_sample.names = getNames();
    addVariablePrefix(default_data_sample.names);
    ROS_ASSERT(areVariableNamesValid(default_data_sample.names));
    // write variable names onto param server
    ros::NodeHandle private_node_handle(recorder_io_.node_handle_, task_recorder_specification.class_name);
    ROS_VERIFY(usc_utilities::write(private_node_handle, "variable_names", default_data_sample.names));
    default_data_sample.data.resize(default_data_sample.names.size(), 0.0);
    message_buffer_.reset(new task_recorder2_utilities::MessageRingBuffer(default_data_sample));

    // do this last
    if (task_recorder_specification.message_timer_rate < 0.0) // record messages by subscribing to the topic
    {
      // if (!is_filtered_)
      // {
      message_subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE, &TaskRecorder<MessageType>::recordMessagesCallback, this);
      // }
    }
    else // record messages at a fixed rate (polling)
    {
      message_timer_ = recorder_io_.node_handle_.createTimer(ros::Duration(1.0/task_recorder_specification.message_timer_rate), &TaskRecorder<MessageType>::recordTimerCallback, this);
      ROS_VERIFY(usc_utilities::read(recorder_io_.node_handle_, "base_frame_id", data_sample_.header.frame_id));
    }

    start_recording_service_server_ = recorder_io_.node_handle_.advertiseService(std::string("start_recording_") + task_recorder_specification.service_prefix + full_topic_name,
                                                                                 &TaskRecorder<MessageType>::startRecording, this);
    stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService(std::string("stop_recording_") + task_recorder_specification.service_prefix + full_topic_name,
                                                                                &TaskRecorder<MessageType>::stopRecording, this);

    // async_spinner_->start();
    return true;
  }

template<class MessageType>
bool TaskRecorder<MessageType>::areVariableNamesValid(const std::vector<std::string>& variable_names,
                                                        const unsigned int maximum_length)
  {
    bool valid = true;
    std::vector<std::string> forbidden_symbols;
    forbidden_symbols.push_back("+");
    forbidden_symbols.push_back("-");
    forbidden_symbols.push_back("*");
    forbidden_symbols.push_back("/");
    forbidden_symbols.push_back("\"");
    forbidden_symbols.push_back("\\");
    forbidden_symbols.push_back("'");
    forbidden_symbols.push_back("(");
    forbidden_symbols.push_back(")");
    forbidden_symbols.push_back("]");
    forbidden_symbols.push_back("[");
    forbidden_symbols.push_back("%");
    forbidden_symbols.push_back("=");
    for (unsigned int i = 0; i < variable_names.size(); ++i)
    {
      unsigned int variable_name_length = variable_names[i].length();
      if (variable_name_length > maximum_length)
      {
        ROS_ERROR("Variable >%s< is of length >%i< and therefore exceeds limit >%i<.",
                  variable_names[i].c_str(), (int)variable_name_length, (int)maximum_length);
        valid = false;
      }
      for (unsigned int n = 0; n < forbidden_symbols.size(); ++n)
      {
        if (variable_names[i].find(forbidden_symbols[n]) != std::string::npos)
        {
          ROS_ERROR("Variable >%s< contains forbidden symbol >%s<.", variable_names[i].c_str(), forbidden_symbols[n].c_str());
          valid = false;
        }
      }
    }
    return valid;
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
    data_sample_.header.stamp = timer_event.current_expected;
    processDataSample(empty_msg);
  }

template<class MessageType>
  void TaskRecorder<MessageType>::processDataSample(const MessageType& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(is_recording_ || is_streaming_)
    {
      if(!transformMsg(msg, data_sample_))
      {
        return;
      }
      if(first_time_)
      {
        data_sample_.names = getNames();
        addVariablePrefix(data_sample_.names);
        ROS_ASSERT(areVariableNamesValid(data_sample_.names));
        first_time_ = false;
      }
      // ROS_VERIFY(filter(data_sample_));
    }
    if (is_recording_)
    {
      recorder_io_.messages_.push_back(data_sample_);
    }
    if (is_streaming_)
    {
      ROS_VERIFY(message_buffer_->add(data_sample_));
    }
  }

template<class MessageType>
  void TaskRecorder<MessageType>::waitForMessage(const unsigned int num_messages_already_received,
                                                 ros::Time& stamp,
                                                 const bool update_abs_start_time)
  {
    // wait for 1st message.
    bool msg_received = false;
    // ROS_DEBUG("Waiting for message >%s<", recorder_io_.topic_name_.c_str());
    unsigned int counter = 0;
    const unsigned int MESSAGE_INDEX = num_messages_already_received + 1;
    while (!msg_received && ros::ok())
    {
      {
        boost::mutex::scoped_lock lock(mutex_);
        // wait for 2 messages to avoid issues when computing dt
        msg_received = (recorder_io_.messages_.size() > MESSAGE_INDEX);
        if (msg_received)
        {
          stamp = recorder_io_.messages_[MESSAGE_INDEX].header.stamp;
          if (update_abs_start_time || stamp < abs_start_time_)
          {
            abs_start_time_ = stamp;
          }
        }
      }
      if (!msg_received)
      {
        ros::Duration(0.01).sleep();
        if (++counter > 100)
        {
          ROS_WARN("Waiting for message on topic >%s<.", recorder_io_.topic_name_.c_str());
          counter = 0;
        }
      }
    }
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::startRecording(const task_recorder2_msgs::Description& description)
  {
    // ROS_DEBUG("Start recording topic named >%s< with description >%s< to id >%i<.",
    //   recorder_io_.topic_name_.c_str(), task_recorder2_utilities::getDescriptionAndId(description).c_str(),
    //   task_recorder2_utilities::getId(description));
    recorder_io_.setDescription(description);
    // if (!is_filtered_)
    // {
    //    message_subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE, &TaskRecorder<MessageType>::recordMessagesCallback, this);
    // }
    {
      boost::mutex::scoped_lock lock(mutex_);
      is_recording_ = true;
      recorder_io_.messages_.clear();
    }
    if (!startRecording())
    {
      ROS_ERROR("Problem starting to record >%s< on topic >%s<.",
                task_recorder2_utilities::getDescriptionAndId(description).c_str(), recorder_io_.topic_name_.c_str());
      return false;
    }
    waitForMessage(0, true);
    // ROS_DEBUG("Recording topic named >%s< with description >%s< to id >%i<.",
    //     recorder_io_.topic_name_.c_str(), task_recorder2_utilities::getDescriptionAndId(description).c_str(),
    //     task_recorder2_utilities::getId(description));
    return true;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::setRecording(const bool recording)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG_COND(is_recording_ && !recording, "Stop recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    ROS_DEBUG_COND(!is_recording_ && recording, "Start recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    is_recording_ = recording;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::setStreaming(const bool streaming)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG_COND(is_streaming_ && !streaming, "Stop streaming topic named >%s<.", recorder_io_.topic_name_.c_str());
    ROS_DEBUG_COND(!is_streaming_ && streaming, "Start streaming topic named >%s<.", recorder_io_.topic_name_.c_str());
    is_streaming_ = streaming;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::getTimeStamps(ros::Time& first,
                                                ros::Time& last,
                                                bool& is_recording,
                                                bool& is_streaming,
                                                unsigned int& num_recorded_messages)
  {
    // first == last if we haven't recorded anything
    first = ros::TIME_MIN;
    last = first;
    boost::mutex::scoped_lock lock(mutex_);
    is_recording = is_recording_;
    is_streaming = is_streaming_;
    num_recorded_messages = recorder_io_.messages_.size();
    bool at_least_one_message_has_been_recorded = (num_recorded_messages > 0);
    if (at_least_one_message_has_been_recorded)
    {
      first = abs_start_time_;
      last = recorder_io_.messages_[num_recorded_messages - 1].header.stamp;
    }
    return at_least_one_message_has_been_recorded;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::startStreaming(task_recorder2_srvs::StartStreaming::Request& request,
                                                 task_recorder2_srvs::StartStreaming::Response& response)
  {
    setStreaming(true);
    response.info = "Started streaming >" + recorder_io_.topic_name_ + "<.";
    response.return_code = response.SERVICE_CALL_SUCCESSFUL;
    ROS_DEBUG_STREAM(response.info);
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::stopStreaming(task_recorder2_srvs::StopStreaming::Request& request,
                                                task_recorder2_srvs::StopStreaming::Response& response)
  {
    setStreaming(false);
    response.info = "Stopped streaming >" + recorder_io_.topic_name_ + "<.";
    response.return_code = response.SERVICE_CALL_SUCCESSFUL;
    ROS_DEBUG_STREAM(response.info);
    return true;
  }


template<class MessageType>
  bool TaskRecorder<MessageType>::startRecording(task_recorder2_srvs::StartRecording::Request& request,
                                                 task_recorder2_srvs::StartRecording::Response& response)
  {
    if (!startRecording(request.description))
    {
      response.info = "Failed to start recording >" + recorder_io_.topic_name_ + "<.";
      ROS_DEBUG_STREAM(response.info);
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
    }
    {
      boost::mutex::scoped_lock lock(mutex_);
      response.start_time = abs_start_time_;
    }
    response.info = ""; // avoid being to verbose
    response.return_code = response.SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
bool TaskRecorder<MessageType>::fail(const std::string& info,
                                     task_recorder2_srvs::StopRecording::Response& response)
  {
    response.info = info;
    ROS_DEBUG_STREAM(response.info);
    response.return_code = response.SERVICE_CALL_FAILED;
    recorder_io_.messages_.clear();
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::stopRecording(task_recorder2_srvs::StopRecording::Request& request,
                                                task_recorder2_srvs::StopRecording::Response& response)
  {
    // error checking
    unsigned int num_messages = 0;
    bool is_recording = isRecording(num_messages);
    if (!is_recording && num_messages == 0)
    {
      return fail("Recorder >" + recorder_io_.topic_name_ + "< is not recording, cannot stop.\n", response);
    }

    // update description and create directories if needed
    task_recorder2_msgs::Description description = request.description;
    if (description.description.empty())
    {
      description = recorder_io_.getDescription();
    }
    recorder_io_.setDescription(description);
    if (recorder_io_.write_out_resampled_data_ && !recorder_io_.createResampledDirectories())
    {
      return fail("Problem creating re-sampled directories for >" + recorder_io_.topic_name_ + "<. Cannot stop recording.\n", response);
    }
    if (recorder_io_.write_out_raw_data_ && !recorder_io_.createRawDirectories())
    {
      return fail("Problem creating raw directories for >" + recorder_io_.topic_name_ + "<. Cannot stop recording.\n", response);
    }

    // stop recording until data is processed and eventually continue afterwards
    setRecording(false);
    if (!filterAndCrop(request.crop_start_time, request.crop_end_time,
                       request.interrupt_start_stamps, request.interrupt_durations,
                       request.num_samples, request.message_names))
    {
      return fail("Could not filter and crop messages of topic >" + recorder_io_.topic_name_ + "<.\n", response);
    }

    // execute in sequence since we need to empty the message buffer afterwards.
    if (recorder_io_.write_out_resampled_data_ && !recorder_io_.writeResampledData())
    {
      return fail("Problem writing re-sampled data.", response);
    }

    // clearing buffer and eventually start recording again
    if (request.return_filtered_and_cropped_messages)
    {
      response.filtered_and_cropped_messages.swap(recorder_io_.messages_);
    }
    recorder_io_.messages_.clear();
    setRecording(!request.stop_recording);
    // if we don't stop logging we need to wait for a message to be safe
    if (!request.stop_recording)
    {
      waitForMessage(0, true);
    }

    response.description = recorder_io_.getDescription();
    response.info = "Stopped recording >" + recorder_io_.topic_name_ + "<.\n";
    response.return_code = response.SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::interruptRecording(task_recorder2_srvs::InterruptRecording::Request& request,
                                                     task_recorder2_srvs::InterruptRecording::Response& response)
  {
    response.info.clear();
    unsigned int num_messages = 0;
    response.was_recording = isRecording(num_messages);
    if (!response.was_recording && num_messages == 0) // up to now, we have not been recording
    {
      response.info = "Cannot continue recording >" + recorder_io_.topic_name_ + "< if it hasn't been started yet.\n";
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
    }
    if (!response.was_recording && !request.recording)
    {
      response.info = "Not recording >" + recorder_io_.topic_name_ + "< and not going to.";
      response.return_code = response.SERVICE_CALL_SUCCESSFUL;
      return true;
    }

    // TODO: this is only needed when actually starting to record, is it ?
    // if (!startRecording())
    // {
    //   response.info = "Problem starting to record >" + task_recorder2_utilities::getDescriptionAndId(recorder_io_.getDescription())
    //   + "< on topic >" + recorder_io_.topic_name_ + "<.";
    //   response.return_code = response.SERVICE_CALL_FAILED;
    //   return true;
    // }

    if (response.was_recording) // we are recording
    {
      if (num_messages > 0) // use the last received stamp
      {
        waitForMessage(num_messages - 1, response.last_recorded_time_stamp, false);
      }
      else // wait for a new one
      {
        waitForMessage(num_messages, response.last_recorded_time_stamp, false);
      }
    }
    setRecording(request.recording);
    if (!response.was_recording) // we were not recording but are now
    {
      waitForMessage(num_messages, response.last_recorded_time_stamp, false);
    }

    // ROS_DEBUG("Last recorded stamp is >%f<.", response.last_recorded_time_stamp.toSec());
    response.return_code = response.SERVICE_CALL_SUCCESSFUL;
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::filterAndCrop(const ros::Time& start_time,
                                                const ros::Time& end_time,
                                                const std::vector<ros::Time>& interrupt_start_stamps,
                                                const std::vector<ros::Duration>& interrupt_durations,
                                                const unsigned int num_samples,
                                                const std::vector<std::string>& message_names)
  {
    std::vector<task_recorder2_msgs::DataSample> filter_and_cropped_messages;
    if(num_samples == 1) // only 1 sample requested
    {
      task_recorder2_msgs::DataSample data_sample;
      ROS_VERIFY(message_buffer_->get(start_time, data_sample));
      data_sample.header.stamp = ros::TIME_MIN;
      filter_and_cropped_messages.push_back(data_sample);
      if (recorder_io_.write_out_raw_data_ && !recorder_io_.writeRawData())
      {
        ROS_ERROR("Could not write raw data.");
        return false;
      }
      recorder_io_.messages_.swap(filter_and_cropped_messages);
      return true;
    }

    // ensure that there is not jump in time caused by interrupting and continuing the recording
    ros::Time new_end_time = end_time;
    if (!interrupt_start_stamps.empty())
    {
      if (interrupt_start_stamps.size() > interrupt_durations.size())
      {
        if (new_end_time > interrupt_start_stamps.back())
        {
          ROS_ERROR("Requested times have not been recorded!");
          ROS_ERROR_STREAM("Requested end time is " << new_end_time << " but recorded end time is " << interrupt_start_stamps.back());
          return false;
        }
      }
      if (!interrupt_durations.empty())
      {
        ros::Duration interrupt_offset(0,0);
        unsigned int interrupt_index = 0;
        bool check_for_interrupts_done = false;
        for (unsigned int i = 0; i < recorder_io_.messages_.size(); ++i)
        {
          recorder_io_.messages_[i].header.stamp = recorder_io_.messages_[i].header.stamp - interrupt_offset;
          if (!check_for_interrupts_done
              && recorder_io_.messages_[i].header.stamp >= interrupt_start_stamps[interrupt_index])
          {
            interrupt_offset += interrupt_durations[interrupt_index];
            if (++interrupt_index >= interrupt_durations.size())
              check_for_interrupts_done = true;
          }
        }
        new_end_time -= interrupt_offset;
        if (new_end_time <= start_time)
        {
          ROS_ERROR("New end time >%f< is smaller than start time >%f< after off-setting for >%f< seconds to account "
              "for interrupts. The last interrupt start stamp was at >%f<.",
              new_end_time.toSec(), start_time.toSec(), interrupt_offset.toSec(), interrupt_start_stamps.back().toSec());
          return false;
        }
      }
    }

    const unsigned int NUM_MESSAGES = recorder_io_.messages_.size();
    if (NUM_MESSAGES == 0)
    {
      ROS_ERROR("Zero messages have been logged.");
      return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_.front().header.stamp;
    ros::Time our_end_time = recorder_io_.messages_.back().header.stamp;

    unsigned int index = 0;
    while (our_end_time.toSec() < 1e-6)
    {
      index++;
      if ((1 + index) > NUM_MESSAGES)
      {
        ROS_ERROR("Time stamps of recorded messages seem to be invalid.");
        return false;
      }
      our_end_time = recorder_io_.messages_[NUM_MESSAGES - (1 + index)].header.stamp;
    }

    if (our_start_time > start_time || our_end_time < new_end_time)
    {
      ROS_ERROR("Requested times have not been recorded!");
      ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
      ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << new_end_time);
      return false;
    }

    // first crop
    if (!task_recorder2_utilities::crop<task_recorder2_msgs::DataSample>(recorder_io_.messages_, start_time, new_end_time))
      return false;
    // then remove duplicates
    if (!task_recorder2_utilities::removeDuplicates<task_recorder2_msgs::DataSample>(recorder_io_.messages_))
      return false;

    if (recorder_io_.write_out_raw_data_ && !recorder_io_.writeRawData())
    {
      ROS_ERROR("Could not write raw data.");
      return false;
    }

    // ROS_DEBUG("Re-sampling >%i< messages to >%i< messages for topic >%s<.",
    //          (int)recorder_io_.messages_.size(), (int)num_samples, recorder_io_.topic_name_.c_str());

    // then re-sample
    if (!resample(recorder_io_.messages_, start_time, new_end_time, num_samples, message_names, filter_and_cropped_messages))
    {
      ROS_ERROR("Could not re-sample data for topic >%s<.", recorder_io_.topic_name_.c_str());
      return false;
    }
    ROS_ASSERT(filter_and_cropped_messages.size() == num_samples);

    recorder_io_.messages_.swap(filter_and_cropped_messages);
    return true;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::resample(std::vector<task_recorder2_msgs::DataSample>& messages,
                                           const ros::Time& start_time,
                                           const ros::Time& end_time,
                                           const unsigned int num_samples,
                                           const std::vector<std::string>& message_names,
                                           std::vector<task_recorder2_msgs::DataSample>& resampled_messages)
  {
  // error checking
    if (messages.size() < 2)
    {
      ROS_ERROR("Need at least 2 messages to re-sample. Cannot re-sample >%s<.", recorder_io_.topic_name_.c_str());
      return false;
    }
    // more error checking
    for (unsigned int i = 0; i < messages.size(); ++i)
    {
      ROS_ASSERT(!messages[i].names.empty());
      ROS_ASSERT(messages[i].names.size() == messages[i].data.size());
    }
    ROS_ASSERT(num_samples > 1);

    // extract indices
    std::vector<int> indices;
    if (message_names.empty())
    {
      ROS_VERIFY(task_recorder2_utilities::getIndices(messages[0].names, messages[0].names, indices));
    }
    else
    {
      ROS_VERIFY(task_recorder2_utilities::getIndices(messages[0].names, message_names, indices));
    }

    ROS_ASSERT(!indices.empty());
    ROS_ASSERT(indices.size() == messages[0].names.size());

    const unsigned int NUM_MESSAGES = messages.size();
    const unsigned int NUM_VARS = indices.size();

    // compute mean dt of the provided time stamps
    double dts[NUM_MESSAGES - 1];
    double mean_dt = 0.0;

    std::vector<double> input_vector(NUM_MESSAGES);
    ros::Time first_time_stamp = messages[0].header.stamp;
    input_vector[0] = messages[0].header.stamp.toSec();
    for (unsigned int i = 0; i < NUM_MESSAGES - 1; ++i)
    {
      dts[i] = messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec();
      mean_dt += dts[i];
      input_vector[i + 1] = input_vector[i] + dts[i];
    }
    mean_dt /= static_cast<double> (NUM_MESSAGES - 1);

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));
    double wave_length = interval.toSec() * static_cast<double> (2.0);

    std::vector<std::vector<double> > variables;
    variables.resize(NUM_VARS);
    for (unsigned int j = 0; j < NUM_MESSAGES; ++j)
    {
      for (unsigned int i = 0; i < NUM_VARS; ++i)
      {
        variables[i].push_back(messages[j].data[indices[i]]);
      }
    }
    std::vector<std::string> names(NUM_VARS);
    for (unsigned int i = 0; i < NUM_VARS; ++i)
    {
      names[i] = messages[0].names[indices[i]];
    }

    task_recorder2_msgs::DataSample data_sample;
    data_sample.header.frame_id = messages[0].header.frame_id;
    data_sample.names = names;
    data_sample.data.resize(data_sample.names.size(), 0.0);
    resampled_messages.resize(num_samples, data_sample);
    std::vector<double> input_querry(num_samples, 0.0);
    for (unsigned int i = 0; i < num_samples; ++i)
    {
      resampled_messages[i].header.seq = i;
      resampled_messages[i].header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
      input_querry[i] = resampled_messages[i].header.stamp.toSec();
    }

    std::vector<std::vector<double> > variables_resampled;
    variables_resampled.resize(NUM_VARS);
    for (unsigned int i = 0; i < NUM_VARS; ++i)
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
          ROS_ERROR("Unknown sampling method for task recorder with topic >%s<. This should never happen.", recorder_io_.topic_name_.c_str());
          return false;
          break;
        }
      }
    }

    // set data and update stamp
    for (unsigned int j = 0; j < num_samples; ++j)
    {
      for (unsigned int i = 0; i < NUM_VARS; ++i)
      {
        resampled_messages[j].data[i] = variables_resampled[i][j];
      }
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

//template<class MessageType>
//  bool TaskRecorder<MessageType>::filter(task_recorder2_msgs::DataSample& data_sample)
//  {
//    // skip filtering if no filter has been specified
//    if (!is_filtered_)
//    {
//      return true;
//    }
//    unfiltered_data_ = data_sample.data;
//    ROS_VERIFY(filter_.update(unfiltered_data_, filtered_data_));
//    data_sample.data = filtered_data_;
//    return true;
//  }

template<class MessageType>
  bool TaskRecorder<MessageType>::getSampleData(const ros::Time& time,
                                                task_recorder2_msgs::DataSample& data_sample)
  {
    // boost::mutex::scoped_lock lock(mutex_);
    if (is_streaming_)
    {
      return message_buffer_->get(time, data_sample);
    }
    // ROS_ERROR("Not steaming.");
    return is_streaming_;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::isRecording()
  {
    bool is_recording;
    boost::mutex::scoped_lock lock(mutex_);
    is_recording = is_recording_;
    return is_recording;
  }
template<class MessageType>
  bool TaskRecorder<MessageType>::isRecording(unsigned int& num_messages)
  {
    bool is_recording;
    boost::mutex::scoped_lock lock(mutex_);
    is_recording = is_recording_;
    num_messages = recorder_io_.messages_.size();
    return is_recording;
  }
template<class MessageType>
  bool TaskRecorder<MessageType>::isStreaming()
  {
  bool is_streaming;
  boost::mutex::scoped_lock lock(mutex_);
  is_streaming = is_streaming_;
    return is_streaming;
  }

}

#endif /* TASK_RECORDER_H_ */
