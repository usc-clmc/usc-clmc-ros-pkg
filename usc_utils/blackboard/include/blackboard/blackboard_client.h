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

  void debug(const std::string& key, const std::string& value);
  void info(const std::string& key, const std::string& value);
  void warn(const std::string& key, const std::string& value);
  void error(const std::string& key, const std::string& value);
  void fatal(const std::string& key, const std::string& value);

  void debug(const std::string& key);
  void info(const std::string& key);
  void warn(const std::string& key);
  void error(const std::string& key);
  void fatal(const std::string& key);

  void startRecording();
  void startStreaming();

  void recording();
  void streaming();
  void logging();

  void stopRecording();
  void interruptRecording();
  void continueRecording();
  void stopStreaming();

  void recordingStopped();
  void recordingInterrupted();
  void recordingContinued();
  void streamingStopped();
  void loggingStopped();

  void recordingFailed();
  void streamingFailed();

  void activatingControl();
  void freezingControl();

  void controlActive();
  void controlFrozen();

  void setupDebug(const std::string& value);
  void setupInfo(const std::string& value);
  void setupWarn(const std::string& value);
  void setupError(const std::string& value);
  void setupFatal(const std::string& value);

  void setup(const std::string& value);

  void setRecording(const bool is_recording);
  void setStreaming(const bool is_streaming);
  void setLogging(const bool is_logging);
  void setLoggingUnknown();

  void description(const std::string& value);

  void descriptionRunning(const std::string& value);
  void descriptionFinished(const std::string& value);
  void descriptionSwapped(const std::string& value);
  void descriptionFailed(const std::string& value);
  void descriptionStopped(const std::string& value);
  void descriptionContinued(const std::string& value);

  void predictionCurrent(const std::string& value);
  void predictionCurrentProgress(const float value);
  void predictionCurrentProbability(const float value);
  void predictionCurrentCalibrated(const float value);
  void predictionSuccessor(const std::string& value);
  void predictionSuccessorProgress(const float value);
  void predictionSuccessorProbability(const float value);
  void predictionSuccessorCalibrated(const float value);
  void predictionFiltered(const std::string& value);
  void predictionProgressFiltered(const float value);

  void leftProgress(const float value);
  void rightProgress(const float value);

private:

  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;

  std::string board_;
  void publish(const std::string& key, const std::string& value, const int color);
  void publish(const std::string& key, const int color);
  void publish(const std::string& key, const std::string& value);

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
