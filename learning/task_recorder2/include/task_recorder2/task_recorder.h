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
// #include <filters/transfer_function.h>

#include <roscpp_utilities/assert.h>
#include <roscpp_utilities/param_server.h>

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
#include <task_recorder2/interpolation.h>

namespace task_recorder2
{

// static const uint32_t ROS_TIME_OFFSET = 1384600000;

template<class MessageType>
  class TaskRecorder : public TaskRecorderBase
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
    typedef boost::shared_ptr<MessageType> MessageTypePtr;

    /*! Constructor
     */
    TaskRecorder();
    /*! Destructor
     */
    virtual ~TaskRecorder();

    /*!
     * @param topic_name
     * @param service_prefix
     * @param variable_name_prefix
     * @return True on success, otherwise False
     */
    bool initialize(const std::string topic_name,
                    const std::string service_prefix = "",
                    const std::string variable_name_prefix = "")
    {
      task_recorder2_msgs::TaskRecorderSpecification task_recorder_specification;
      task_recorder_specification.topic_name = topic_name;
      task_recorder_specification.service_prefix = service_prefix;
      task_recorder_specification.variable_name_prefix = variable_name_prefix;
      return initialize(task_recorder_specification);
    }

    /*!
     * @param task_recorder_specification
     * @return True on success, otherwise False
     */
    bool initialize(const task_recorder2_msgs::TaskRecorderSpecification& task_recorder_specification);

    /*!
     * @param node_handle
     * @param class_name
     * @param class_name_prefix
     * @return True on success, otherwise False
     */
    bool readParams(ros::NodeHandle& node_handle,
                    const std::string& class_name,
                    const std::string& class_name_prefix);

    /*!
     * @return prefixed variable names
     */
    std::vector<std::string> getPrefixedNames() const
    {
      std::vector<std::string> names = getNames();
      addVariablePrefix(names);
      ROS_ASSERT(areVariableNamesValid(names));
      return names;
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

    bool usesTimer() const
    {
      return uses_timer_;
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
    virtual bool transformMsg(const MessageTypeConstPtr message,
                              task_recorder2_msgs::DataSample& data_sample_msg) = 0;

    /*!
     * @return Number of signals
     */
    virtual unsigned int getNumSignals() const = 0;

    /*!
     * @param node_handle The node_handle that is specific to the (particular) task recorder
     * Derived classes can implement this function to retrieve (arm) specific parameters
     * @return True on success, otherwise False
     */
    virtual bool readParams(ros::NodeHandle& node_handle)
    {
      return true;
    }

    /*!
     * @return All the variable names that the task recorder can record
     */
    virtual std::vector<std::string> getNames() const = 0;

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

    /*! True if the recorder uses its own timer to record messages
     * False if the recorder listens to a topic
     */
    bool uses_timer_;
    double recording_rate_;

    /*!
     */
    unsigned int number_of_signals_;

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
    ros::Time abs_start_time_;

    /*! offset which has been set by controller manager and is read from param server
     * the purpose is to keep the time in a range that is suitable for storing as a float
     */
    uint32_t ros_time_offset_;

    /*!
     */
    typedef boost::shared_mutex Lock;
    typedef boost::unique_lock< Lock > WriteLock;
    typedef boost::shared_lock< Lock > ReadLock;

    /*!
     */
    Lock data_mutex_;

    /*!
     */
    Lock is_recording_mutex_;
    bool is_recording_;
    Lock is_streaming_mutex_;
    bool is_streaming_;

    /*! Used for streaming. The buffer never gets deleted so no mutex required
     */
    boost::shared_ptr<task_recorder2_utilities::MessageRingBuffer> message_buffer_;

    /*!
     */
    task_recorder2_msgs::DataSample data_sample_;
    task_recorder2_msgs::DataSample message_buffer_data_sample_;
    MessageTypePtr message_;
    std::vector<task_recorder2_msgs::data_sample_scalar> filtered_data_;

    /*! Check whether variable names do not exceed maximum length (required by CLMC plot and friends)
     * @param variable_names
     * @return True on success, otherwise False
     */
    bool areVariableNamesValid(const std::vector<std::string>& variable_names,
                                        const unsigned int maximum_length = 20) const;

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
     * @param start_time
     * @return True on success, otherwise False
     */
    bool startRecorder(const task_recorder2_msgs::Description& description,
                       ros::Time& start_time);

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

    /*!
     */
    double filter_coefficient_;
    /*!
     * @param data_sample
     */
    void filter(task_recorder2_msgs::DataSample& data_sample);

    /*!
     * @param message
     */
    void messageCallback(const MessageTypeConstPtr message);
    /*!
     * @param timer_event
     */
    void timerCallback(const ros::TimerEvent& timer_event);
    /*!
     * @param message
     * @param stamp
     */
    void processDataSample(const MessageTypeConstPtr message, const ros::Time& stamp);

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
    void addVariablePrefix(std::vector<std::string>& names) const
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
    first_time_(true), recorder_io_(ros::NodeHandle("/TaskRecorderManager")),
    uses_timer_(false), recording_rate_(0.0), number_of_signals_(0), variable_name_prefix_(""),
    ros_time_offset_(0), is_recording_(false), is_streaming_(false),
    splining_method_(Linear), filter_coefficient_(0.0)
  {
  }

template<class MessageType>
  TaskRecorder<MessageType>::~TaskRecorder()
  {
  }

template<typename MessageType>
struct remove_pointer
{
    typedef MessageType type;
};

template<typename MessageType>
struct remove_pointer<MessageType*>
{
    typedef MessageType type;
};

template<class MessageType>
  bool TaskRecorder<MessageType>::initialize(const task_recorder2_msgs::TaskRecorderSpecification& task_recorder_specification)
  {
    if (!recorder_io_.initialize(task_recorder_specification.topic_name, task_recorder_specification.service_prefix))
    {
      ROS_ERROR("Could not initialize task recorder on topic >%s< with prefix >%s<.",
                task_recorder_specification.topic_name.c_str(), task_recorder_specification.service_prefix.c_str());
      return false;
    }

    std::string full_topic_name = task_recorder_specification.topic_name;
    if (!task_recorder2_utilities::getTopicName(full_topic_name))
    {
      ROS_ERROR("Could not obtain topic name from name >%s<. Could not initialize base of the task recorder.", full_topic_name.c_str());
      return false;
    }

    number_of_signals_ = getNumSignals();
    data_sample_.data.resize(number_of_signals_, 0.0);
    data_sample_.names.resize(number_of_signals_);
    message_buffer_data_sample_.data.resize(number_of_signals_, 0.0);
    message_buffer_data_sample_.names.resize(number_of_signals_);

    filtered_data_.resize(number_of_signals_, 0.0);
    double sampling_rate = 0.0;
    if (!usc_utilities::read(recorder_io_.node_handle_, "sampling_rate", sampling_rate))
      return false;

    filter_coefficient_ = 0.0; // do not filter the data
    if (sampling_rate < recording_rate_)
    {
      const int period = static_cast<int>(floor(recording_rate_ / sampling_rate));
      filter_coefficient_ = 2.0 / (static_cast<double>(period) + 1.0);
    }

    task_recorder2_msgs::DataSample default_data_sample;
    default_data_sample.header.frame_id = "                                                              "; // something big
    default_data_sample.names = getPrefixedNames();
    // write variable names onto parameter server
    ros::NodeHandle private_node_handle(recorder_io_.node_handle_, task_recorder_specification.class_name);
    if (!usc_utilities::write(private_node_handle, "variable_names", default_data_sample.names))
      return false;
    default_data_sample.data.resize(default_data_sample.names.size(), 0.0);
    message_buffer_.reset(new task_recorder2_utilities::MessageRingBuffer(default_data_sample));

    // do this last
    if (uses_timer_) // record messages at a fixed rate (polling)
    {
      typedef typename remove_pointer<MessageType>::type type;
      message_.reset(new type());
      message_timer_ = recorder_io_.node_handle_.createTimer(
          ros::Duration(1.0/recording_rate_), &TaskRecorder<MessageType>::timerCallback, this);
      if (!usc_utilities::read(recorder_io_.node_handle_, "base_frame_id", data_sample_.header.frame_id))
        return false;
      message_buffer_data_sample_.header.frame_id = data_sample_.header.frame_id;
    }
    else // record messages by subscribing to the topic
    {
      message_subscriber_ = recorder_io_.node_handle_.subscribe(
          recorder_io_.topic_name_, MESSAGE_SUBSCRIBER_BUFFER_SIZE,
          &TaskRecorder<MessageType>::messageCallback, this);
    }

    start_recording_service_server_ = recorder_io_.node_handle_.advertiseService(
        "start_recording_" + task_recorder_specification.service_prefix + full_topic_name,
        &TaskRecorder<MessageType>::startRecording, this);
    stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService(
        "stop_recording_" + task_recorder_specification.service_prefix + full_topic_name,
        &TaskRecorder<MessageType>::stopRecording, this);

    if (!usc_utilities::read(recorder_io_.node_handle_, "/SL/ros_time_offset", ros_time_offset_))
    {
      ROS_ERROR("Could not read >/SL/ros_time_offset<. Is SL started yet once.");
      return false;
    }

    return true;
  }

template<class MessageType>
bool TaskRecorder<MessageType>::areVariableNamesValid(const std::vector<std::string>& variable_names,
                                                        const unsigned int maximum_length) const
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
      ROS_ERROR("Unknown splining method >%s<. Cannot initialize task recorder with topic >%s<.",
                splining_method.c_str(), splining_method.c_str());
      return false;
    }
    return true;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::messageCallback(const MessageTypeConstPtr message)
  {
    processDataSample(message, message->header.stamp);
  }

template<class MessageType>
  void TaskRecorder<MessageType>::timerCallback(const ros::TimerEvent& timer_event)
  {
    processDataSample(message_, timer_event.current_real);
  }

template<class MessageType>
  void TaskRecorder<MessageType>::processDataSample(const MessageTypeConstPtr message,
                                                    const ros::Time& stamp)
  {
    {
      WriteLock lock(data_mutex_);
      data_sample_.header.stamp = stamp;
      if (!transformMsg(message, data_sample_))
      {
        return;
      }
      if (first_time_)
      {
        filtered_data_ = data_sample_.data; // initialize filter
        data_sample_.names = getPrefixedNames();
        message_buffer_data_sample_.names = data_sample_.names;
        message_buffer_data_sample_.header = data_sample_.header;
        first_time_ = false; // do this last
      }
      filter(data_sample_);
      message_buffer_data_sample_.header.stamp = data_sample_.header.stamp;
      message_buffer_data_sample_.data = data_sample_.data;
      if (isRecording())
      {
        recorder_io_.messages_.push_back(data_sample_);
      }
    }
    if (isStreaming())
    {
      message_buffer_->add(message_buffer_data_sample_);
    }
  }

template<class MessageType>
  void TaskRecorder<MessageType>::waitForMessage(const unsigned int num_messages_already_received,
                                                 ros::Time& stamp,
                                                 const bool update_abs_start_time)
  {
    // wait for the first 2 (num_messages_already_received + 1) message.
    bool msg_received = false;
    unsigned int counter = 0;
    const unsigned int MESSAGE_INDEX = num_messages_already_received + 1;
    while (!msg_received && ros::ok())
    {
      {
        ReadLock lock(data_mutex_);
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
  bool TaskRecorder<MessageType>::startRecorder(const task_recorder2_msgs::Description& description, ros::Time& start_time)
  {
    // ROS_DEBUG("Start recording topic named >%s< with description >%s< to id >%i<.",
    // recorder_io_.topic_name_.c_str(), task_recorder2_utilities::getDescriptionAndId(description).c_str(),
    // task_recorder2_utilities::getId(description));
    recorder_io_.setDescription(description);
    setRecording(true);
    {
      WriteLock lock(data_mutex_);
      recorder_io_.messages_.clear();
    }
    if (!startRecording())
    {
      ROS_ERROR("Problem starting to record >%s< on topic >%s<.",
                task_recorder2_utilities::getDescriptionAndId(description).c_str(), recorder_io_.topic_name_.c_str());
      return false;
    }
    waitForMessage(0, true);
    {
      WriteLock lock(data_mutex_);
      start_time = abs_start_time_;
    }
    // ROS_DEBUG("Recording topic named >%s< with description >%s< to id >%i<.",
    // recorder_io_.topic_name_.c_str(), task_recorder2_utilities::getDescriptionAndId(description).c_str(),
    // task_recorder2_utilities::getId(description));
    return true;
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
    is_recording = isRecording();
    is_streaming = isStreaming();
    ReadLock lock(data_mutex_);
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
  bool TaskRecorder<MessageType>::readParams(ros::NodeHandle& node_handle,
                const std::string& class_name,
                const std::string& class_name_prefix)
  {
    variable_name_prefix_ = "";
    if (!class_name_prefix.empty())
    {
      ros::NodeHandle private_node_handle(recorder_io_.node_handle_, class_name_prefix);
      if (private_node_handle.hasParam("variable_name_prefix"))
      {
        if (!usc_utilities::read(private_node_handle, "variable_name_prefix", variable_name_prefix_))
          return false;
      }
    }

    ros::NodeHandle class_private_node_handle(recorder_io_.node_handle_, class_name);
    // setting default values
    splining_method_ = Linear;
    if (class_private_node_handle.hasParam("splining_method"))
    {
      std::string splining_method = "";
      if (!usc_utilities::read(class_private_node_handle, "splining_method", splining_method))
        return false;
      if (!setSpliningMethod(splining_method))
        return false;
    }
    const std::string EXPECTED_RATE = "expected_rate";
    const std::string MESSAGE_TIMER_RATE = "message_timer_rate";
    if (class_private_node_handle.hasParam(EXPECTED_RATE)
        && class_private_node_handle.hasParam(MESSAGE_TIMER_RATE))
    {
      ROS_ERROR("Recorder class >%s< has both, an expected rate and a message timer rate specified "
          "in namespace >%s<. That does not make sense.",
          class_private_node_handle.getNamespace().c_str(), class_name.c_str());
      return false;
    }
    else if (class_private_node_handle.hasParam(EXPECTED_RATE))
    {
      uses_timer_ = false;
      if (!usc_utilities::read(class_private_node_handle, EXPECTED_RATE, recording_rate_))
        return false;
    }
    else if (class_private_node_handle.hasParam(MESSAGE_TIMER_RATE))
    {
      uses_timer_ = true;
      if (!usc_utilities::read(class_private_node_handle, MESSAGE_TIMER_RATE, recording_rate_))
        return false;
    }
    else
    {
      ROS_ERROR("Recorder class >%s< need to either specify an expected "
          "rate or a message timer rate in namespace >%s<.",
          class_name.c_str(), class_private_node_handle.getNamespace().c_str());
      return false;
    }
    return readParams(node_handle);
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
    if (!startRecorder(request.description, response.start_time))
    {
      response.info = "Failed to start recording >" + recorder_io_.topic_name_ + "<.";
      ROS_DEBUG_STREAM(response.info);
      response.return_code = response.SERVICE_CALL_FAILED;
      return true;
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
      {
        if (!message_buffer_->get(start_time, data_sample))
        {
          ROS_ERROR("Could not get sample from message buffer.");
          return false;
        }
      }
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

    std::vector<ros::Time> updated_interrupt_start_stamps = interrupt_start_stamps;

    // ensure that there is not jump in time caused by interrupting and continuing the recording
    ros::Time new_end_time = end_time;
    if (!updated_interrupt_start_stamps.empty())
    {
      if (updated_interrupt_start_stamps.size() > interrupt_durations.size())
      {
        if (new_end_time > updated_interrupt_start_stamps.back())
        {
          ROS_ERROR("Requested times have not been recorded!");
          ROS_ERROR_STREAM("Requested end time is " << new_end_time << " but recorded end time is " << updated_interrupt_start_stamps.back());
          return false;
        }
      }

      // TODO: fix this hack
      if (!interrupt_durations.empty())
      {
        ros::Duration interrupt_offset(0,0);
        unsigned int interrupt_index = 0;
        bool check_for_interrupts_done = false;
        for (unsigned int i = 0; i < recorder_io_.messages_.size(); ++i)
        {
          recorder_io_.messages_[i].header.stamp -= interrupt_offset;

          if (!check_for_interrupts_done
              && recorder_io_.messages_[i].header.stamp >= updated_interrupt_start_stamps[interrupt_index])
          {
            interrupt_offset += interrupt_durations[interrupt_index];
            // ROS_WARN("Updating interrupt_offset after (%i) to %.4f", (int)i, interrupt_offset.toSec());
            interrupt_index++;
            for (unsigned int j = interrupt_index; j < updated_interrupt_start_stamps.size(); ++j)
            {
              // ROS_WARN("Before updating updated_interrupt_start_stamps[%i] from %.4f", (int)i, updated_interrupt_start_stamps[j].toSec());
              updated_interrupt_start_stamps[j] -= interrupt_durations[interrupt_index - 1];
              // ROS_ERROR("After updating updated_interrupt_start_stamps[%i] to %.4f", (int)i, updated_interrupt_start_stamps[j].toSec());
            }

            if (interrupt_index >= interrupt_durations.size())
            {
              // ROS_INFO("Done checking, interrupt_index is >%i<.", (int)interrupt_index);
              check_for_interrupts_done = true;
            }
          }
        }
        new_end_time -= interrupt_offset;
        if (new_end_time <= start_time)
        {
          ROS_ERROR("New end time >%f< is smaller than start time >%f< after off-setting for >%f< seconds to account "
              "for interrupts. The last interrupt start stamp was at >%f<.",
              new_end_time.toSec(), start_time.toSec(), interrupt_offset.toSec(), updated_interrupt_start_stamps.back().toSec());
          return false;
        }
      }
    }

    // TODO: fix this hack
    bool done = false;
    while (!done)
    {
      done = true;
      for (unsigned int i = 1; i < recorder_io_.messages_.size(); ++i)
      {
        double dt = recorder_io_.messages_[i].header.stamp.toSec() - recorder_io_.messages_[i-1].header.stamp.toSec();
        if (dt < 0.0)
        {
          // ROS_FATAL("Removing index %i = %.4f", (int)i, recorder_io_.messages_[i].header.stamp.toSec());
          recorder_io_.messages_.erase (recorder_io_.messages_.begin() + i);
          done = false;
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
    {
      ROS_ERROR("Problems cropping >%i< messages starting at >%f< and ending at >%f<.",
                (int)recorder_io_.messages_.size(), start_time.toSec(), new_end_time.toSec());
      return false;
    }
    // then remove duplicates
    if (!task_recorder2_utilities::removeDuplicates<task_recorder2_msgs::DataSample>(recorder_io_.messages_))
    {
      ROS_ERROR("Problems removing duplicates from >%i< recorded messages.", (int)recorder_io_.messages_.size());
      return false;
    }

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

    // check whether we run out of single precision
    double double_stamp = (first_time_stamp - ros::Duration(ros_time_offset_)).toSec();
    float float_stamp = static_cast<float>((first_time_stamp - ros::Duration(ros_time_offset_)).toSec());
    if (fabs(double_stamp - float_stamp) > 10e-3)
    {
      ROS_ERROR("Double precision needed to capture time stamps.");
      ROS_ERROR("Consider to update ROS_TIME_OFFSET in task_recorder2.");
      ROS_ERROR("Current time is (in seconds) >%u<.", first_time_stamp.sec);
      ROS_ERROR("Make sure to also update SL.");
      return false;
    }

    input_vector[0] = messages[0].header.stamp.toSec();
    for (unsigned int i = 0; i < NUM_MESSAGES - 1; ++i)
    {
      dts[i] = messages[i + 1].header.stamp.toSec() - messages[i].header.stamp.toSec();

//      if (dts[i] > 0.1)
//        ROS_WARN("HIERHIEHRI: dts[%i] = %f", (int)i, dts[i]);

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
          if (!resampleBSpline(input_vector, variables[i], wave_length, input_querry, variables_resampled[i], false))
          {
            return false;
          }
          break;
        }
        case Linear:
        {
          if (!resampleLinearNoBounds(input_vector, variables[i], input_querry, variables_resampled[i]))
          {
            return false;
          }
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
      resampled_messages[j].header.stamp = static_cast<ros::Time> (first_time_stamp - ros::Duration(ros_time_offset_) + ros::Duration(j * interval.toSec()));
    }
    return true;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::filter(task_recorder2_msgs::DataSample& data_sample)
  {
    for (unsigned int i = 0; i < data_sample.data.size(); ++i)
    {
      data_sample.data[i] = filter_coefficient_ * filtered_data_[i] + (1.0 - filter_coefficient_) * data_sample.data[i];
      filtered_data_[i] = data_sample.data[i];
    }
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::getSampleData(const ros::Time& time,
                                                task_recorder2_msgs::DataSample& data_sample)
  {
    if (isStreaming())
    {
      return message_buffer_->get(time, data_sample);
    }
    return false;
  }


template<class MessageType>
  void TaskRecorder<MessageType>::setRecording(const bool recording)
  {
  WriteLock lock(is_recording_mutex_);
    ROS_DEBUG_COND(is_recording_ && !recording, "Stop recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    ROS_DEBUG_COND(!is_recording_ && recording, "Start recording topic named >%s<.", recorder_io_.topic_name_.c_str());
    is_recording_ = recording;
  }

template<class MessageType>
  void TaskRecorder<MessageType>::setStreaming(const bool streaming)
  {
    WriteLock lock(is_streaming_mutex_);
    ROS_DEBUG_COND(is_streaming_ && !streaming, "Stop streaming topic named >%s<.", recorder_io_.topic_name_.c_str());
    ROS_DEBUG_COND(!is_streaming_ && streaming, "Start streaming topic named >%s<.", recorder_io_.topic_name_.c_str());
    is_streaming_ = streaming;
  }

template<class MessageType>
  bool TaskRecorder<MessageType>::isRecording()
  {
    bool is_recording = false;
    {
      ReadLock lock(is_recording_mutex_);
      is_recording = is_recording_;
    }
    return is_recording;
  }
template<class MessageType>
  bool TaskRecorder<MessageType>::isRecording(unsigned int& num_messages)
  {
    bool is_recording;
    {
      ReadLock lock(is_recording_mutex_);
      is_recording = is_recording_;
//    }
//    {
      num_messages = recorder_io_.messages_.size();
    }
    return is_recording;
  }
template<class MessageType>
  bool TaskRecorder<MessageType>::isStreaming()
  {
    bool is_streaming;
    ReadLock lock(is_streaming_mutex_);
    is_streaming = is_streaming_;
    return is_streaming;
  }

}

#endif /* TASK_RECORDER_H_ */
