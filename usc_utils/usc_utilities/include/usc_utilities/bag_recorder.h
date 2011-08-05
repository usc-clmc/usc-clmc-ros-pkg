/*
 * bag_recorder.h
 *
 *  Created on: Feb 22, 2011
 *      Author: kalakris
 */

#ifndef USC_UTILITIES_BAG_RECORDER_H_
#define USC_UTILITIES_BAG_RECORDER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <usc_utilities/file_io.h>

namespace usc_utilities
{

template<class MessageType>
class BagRecorder
{
public:
  BagRecorder(const std::string& topic_name, int queue_size);
  virtual ~BagRecorder();

  void startRecording();
  void stopRecording();
  bool saveToBagFile(const std::string& abs_bag_file_name);
  void clear();

  const std::vector<MessageType>& getMessages();

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
  typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
  void callback(const MessageTypeConstPtr& message);

  boost::mutex messages_mutex_;
  std::vector<MessageType> messages_;
  std::string topic_name_;
  bool recording_;
};

template<class MessageType>
BagRecorder<MessageType>::BagRecorder(const std::string& topic_name, int queue_size):
  spinner_(1, &callback_queue_),
  topic_name_(topic_name)
{
  recording_ = false;
  node_handle_.setCallbackQueue(&callback_queue_);
  subscriber_ = node_handle_.subscribe(topic_name, queue_size, &BagRecorder<MessageType>::callback, this);
  spinner_.start();
}

template<class MessageType>
BagRecorder<MessageType>::~BagRecorder()
{
  subscriber_.shutdown();
  spinner_.stop();
}

template<class MessageType>
void BagRecorder<MessageType>::startRecording()
{
  boost::mutex::scoped_lock lock(messages_mutex_);
  recording_ = true;
}

template<class MessageType>
void BagRecorder<MessageType>::stopRecording()
{
  boost::mutex::scoped_lock lock(messages_mutex_);
  recording_ = false;
}

template<class MessageType>
bool BagRecorder<MessageType>::saveToBagFile(const std::string& abs_bag_file_name)
{
  boost::mutex::scoped_lock lock(messages_mutex_);
  return FileIO<MessageType>::writeToBagFileWithTimeStamps(messages_, topic_name_, abs_bag_file_name);
}

template<class MessageType>
void BagRecorder<MessageType>::clear()
{
  boost::mutex::scoped_lock lock(messages_mutex_);
  messages_.clear();
}

template<class MessageType>
void BagRecorder<MessageType>::callback(const MessageTypeConstPtr& message)
{
  boost::mutex::scoped_lock lock(messages_mutex_);
  if (recording_)
    messages_.push_back(*message);
}

template<class MessageType>
const std::vector<MessageType>& BagRecorder<MessageType>::getMessages()
{
  return messages_;
}

}

#endif /* USC_UTILITIES_BAG_RECORDER_H_ */
