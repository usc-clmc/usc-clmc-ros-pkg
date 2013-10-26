/*
 * blackboard_client.h
 *
 *  Created on: Oct 13, 2013
 *      Author: pastor
 */

#ifndef BLACKBOARD_CLIENT_H_
#define BLACKBOARD_CLIENT_H_

#include <ros/ros.h>

#include <string>
#include <blackboard/BlackBoardEntry.h>

namespace blackboard
{

class BlackBoardClient
{

public:

  BlackBoardClient();
  virtual ~BlackBoardClient() {};

  /*!
   * @param board Name of the board. For now this should be either "right", "middle", or "left"
   * @param single_threaded
   * @return True on success, otherwise False
   */
  bool initialize(const std::string& board, const bool single_threaded = false);

  void debug(const std::string& key, const std::string& value)
  {
    publish(key, value, BlackBoardEntry::GREEN);
  }
  void info(const std::string& key, const std::string& value)
  {
    publish(key, value, BlackBoardEntry::WHITE);
  }
  void warn(const std::string& key, const std::string& value)
  {
    publish(key, value, BlackBoardEntry::YELLOW);
  }
  void error(const std::string& key, const std::string& value)
  {
    publish(key, value, BlackBoardEntry::RED);
  }
  void fatal(const std::string& key, const std::string& value)
  {
    publish(key, value, BlackBoardEntry::PURPLE);
  }

  void debug(const std::string& key)
  {
    publish(key, BlackBoardEntry::GREEN);
  }
  void info(const std::string& key)
  {
    publish(key, BlackBoardEntry::WHITE);
  }
  void warn(const std::string& key)
  {
    publish(key, BlackBoardEntry::YELLOW);
  }
  void error(const std::string& key)
  {
    publish(key, BlackBoardEntry::RED);
  }
  void fatal(const std::string& key)
  {
    publish(key, BlackBoardEntry::PURPLE);
  }

  void startRecording()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STARTING_VALUE, BlackBoardEntry::YELLOW);
  }
  void startStreaming()
  {
    publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STARTING_VALUE, BlackBoardEntry::YELLOW);
  }

  void recording()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STARTED_VALUE, BlackBoardEntry::RED);
  }
  void streaming()
  {
    publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STARTED_VALUE, BlackBoardEntry::YELLOW);
  }

  void stopRecording()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STOPPING_VALUE, BlackBoardEntry::YELLOW);
  }
  void interruptRecording()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::INTERRUPTING_VALUE, BlackBoardEntry::YELLOW);
  }
  void continueRecording()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::INTERRUPTING_VALUE, BlackBoardEntry::YELLOW);
  }
  void stopStreaming()
  {
    publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STOPPING_VALUE, BlackBoardEntry::YELLOW);
  }

  void recordingStopped()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STOPPED_VALUE, BlackBoardEntry::WHITE);
  }
  void recordingInterrupted()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::INTERRUPTED_VALUE, BlackBoardEntry::YELLOW);
  }
  void recordingContinued()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::CONTINUED_VALUE, BlackBoardEntry::BLUE);
  }
  void streamingStopped()
  {
    publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STOPPED_VALUE, BlackBoardEntry::WHITE);
  }

  void recordingFailed()
  {
    publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::FAILED_VALUE, BlackBoardEntry::PURPLE);
  }
  void streamingFailed()
  {
    publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::FAILED_VALUE, BlackBoardEntry::PURPLE);
  }

  void activatingControl()
  {
    publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::YELLOW);
  }
  void freezingControl()
  {
    publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::YELLOW);
  }

  void controlActive()
  {
    publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::ACTIVE_VALUE, BlackBoardEntry::GREEN);
  }
  void controlFrozen()
  {
    publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::FROZEN_VALUE, BlackBoardEntry::RED);
  }


  void setupDebug(const std::string& value)
  {
    publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::GREEN);
  }
  void setupInfo(const std::string& value)
  {
    publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::WHITE);
  }
  void setupWarn(const std::string& value)
  {
    publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::YELLOW);
  }
  void setupError(const std::string& value)
  {
    publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::RED);
  }
  void setupFatal(const std::string& value)
  {
    publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::PURPLE);
  }


private:

  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;

  std::string board_;
  void publish(const std::string& key, const std::string& value, const int color);
  void publish(const std::string& key, const int color);

  void reset();

  bool single_threaded_;

};

class RightBlackBoardClient : public BlackBoardClient
{
public:
  RightBlackBoardClient(const bool single_threaded = false);
  virtual ~RightBlackBoardClient() {};
private:

};

class MiddleBlackBoardClient : public BlackBoardClient
{
public:
  MiddleBlackBoardClient(const bool single_threaded = false);
  virtual ~MiddleBlackBoardClient() {};
private:

};

class LeftBlackBoardClient : public BlackBoardClient
{
public:
  LeftBlackBoardClient(const bool single_threaded = false);
  virtual ~LeftBlackBoardClient() {};
private:

};

}


#endif /* BLACKBOARD_CLIENT_H_ */
