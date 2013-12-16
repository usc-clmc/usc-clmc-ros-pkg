/*
 * blackboard_client.cpp
 *
 *  Created on: Oct 13, 2013
 *      Author: pastor
 */

#include <usc_utilities/assert.h>

#include <blackboard/blackboard_client.h>

namespace blackboard
{

BlackBoardClient::BlackBoardClient() :
    node_handle_("/BlackBoard"), board_(""), single_threaded_(false)
{
}

bool BlackBoardClient::initialize(const std::string& board, const bool single_threaded)
{
  board_ = board;
  const int PUBLISHER_BUFFER_SIZE = 10;
  publisher_ = node_handle_.advertise<blackboard::BlackBoardEntry>("entries", PUBLISHER_BUFFER_SIZE);
  ros::Duration(1.0).sleep(); // wait for the publisher to shake hands
  single_threaded_ = single_threaded;
  reset();
  return true;
}

void BlackBoardClient::publish(const std::string& key, const std::string& value, const int color)
{
  BlackBoardEntry entry;
  entry.action = BlackBoardEntry::UPDATE_AND_CHANGE_COLOR;
  entry.board = board_;
  entry.key = key;
  entry.value = value;
  entry.color = color;
  publisher_.publish(entry);
  if (single_threaded_)
  {
    // ROS_DEBUG("Spinning.");
    ros::spinOnce();
  }
}

void BlackBoardClient::publish(const std::string& key, const int color)
{
  BlackBoardEntry entry;
  entry.action = BlackBoardEntry::CHANGE_COLOR;
  entry.board = board_;
  entry.key = key;
  entry.color = color;
  publisher_.publish(entry);
  if (single_threaded_)
  {
    // ROS_DEBUG("Spinning.");
    ros::spinOnce();
  }
}

void BlackBoardClient::publish(const std::string& key, const std::string& value)
{
  BlackBoardEntry entry;
  entry.action = BlackBoardEntry::UPDATE_BUT_KEEP_COLOR;
  entry.color = 0; // not used
  entry.board = board_;
  entry.key = key;
  entry.value = value;
  publisher_.publish(entry);
  if (single_threaded_)
  {
    // ROS_DEBUG("Spinning.");
    ros::spinOnce();
  }
}

void BlackBoardClient::reset()
{
  if (board_ == "right")
  {
    info(BlackBoardEntry::LOGGING_KEY, "");
    info(BlackBoardEntry::STREAMING_KEY, "");
    info(BlackBoardEntry::RECORDING_KEY, "");
    info(BlackBoardEntry::SETUP_KEY, "");
  }
  else if (board_ == "left")
  {
    info(BlackBoardEntry::DESCRIPTION_KEY, "");
    info(BlackBoardEntry::PREDICTION_CURRENT_KEY, "");
    info(BlackBoardEntry::PREDICTION_CURRENT_PROGRESS_KEY, "");
    info(BlackBoardEntry::PREDICTION_CURRENT_PROBABILITY_KEY, "");
    info(BlackBoardEntry::PREDICTION_CURRENT_CALIBRATED_KEY, "");
    info(BlackBoardEntry::PREDICTION_SUCCESSOR_KEY, "");
    info(BlackBoardEntry::PREDICTION_SUCCESSOR_PROGRESS_KEY, "");
    info(BlackBoardEntry::PREDICTION_SUCCESSOR_PROBABILITY_KEY, "");
    info(BlackBoardEntry::PREDICTION_SUCCESSOR_CALIBRATED_KEY, "");
  }
}


RightBlackBoardClient::RightBlackBoardClient(const bool single_threaded)
{
  ROS_VERIFY(initialize("right", single_threaded));
}

LeftBlackBoardClient::LeftBlackBoardClient(const bool single_threaded)
{
  ROS_VERIFY(initialize("left", single_threaded));
}

void BlackBoardClient::debug(const std::string& key, const std::string& value)
{
  publish(key, value, BlackBoardEntry::GREEN);
}
void BlackBoardClient::info(const std::string& key, const std::string& value)
{
  publish(key, value, BlackBoardEntry::WHITE);
}
void BlackBoardClient::warn(const std::string& key, const std::string& value)
{
  publish(key, value, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::error(const std::string& key, const std::string& value)
{
  publish(key, value, BlackBoardEntry::RED);
}
void BlackBoardClient::fatal(const std::string& key, const std::string& value)
{
  publish(key, value, BlackBoardEntry::PURPLE);
}

void BlackBoardClient::debug(const std::string& key)
{
  publish(key, BlackBoardEntry::GREEN);
}
void BlackBoardClient::info(const std::string& key)
{
  publish(key, BlackBoardEntry::WHITE);
}
void BlackBoardClient::warn(const std::string& key)
{
  publish(key, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::error(const std::string& key)
{
  publish(key, BlackBoardEntry::RED);
}
void BlackBoardClient::fatal(const std::string& key)
{
  publish(key, BlackBoardEntry::PURPLE);
}

void BlackBoardClient::startRecording()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STARTING_VALUE, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::startStreaming()
{
  publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STARTING_VALUE, BlackBoardEntry::YELLOW);
}

void BlackBoardClient::recording()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STARTED_VALUE, BlackBoardEntry::RED);
}
void BlackBoardClient::streaming()
{
  publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STARTED_VALUE, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::logging()
{
  publish(BlackBoardEntry::LOGGING_KEY, BlackBoardEntry::ON_VALUE, BlackBoardEntry::GREEN);
}

void BlackBoardClient::stopRecording()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STOPPING_VALUE, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::interruptRecording()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::INTERRUPTING_VALUE, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::continueRecording()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::CONTINUING_VALUE, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::stopStreaming()
{
  publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STOPPING_VALUE, BlackBoardEntry::YELLOW);
}

void BlackBoardClient::recordingStopped()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::STOPPED_VALUE, BlackBoardEntry::WHITE);
}
void BlackBoardClient::recordingInterrupted()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::INTERRUPTED_VALUE, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::recordingContinued()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::CONTINUED_VALUE, BlackBoardEntry::BLUE);
}
void BlackBoardClient::streamingStopped()
{
  publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::STOPPED_VALUE, BlackBoardEntry::WHITE);
}
void BlackBoardClient::loggingStopped()
{
  publish(BlackBoardEntry::LOGGING_KEY, BlackBoardEntry::OFF_VALUE, BlackBoardEntry::WHITE);
}

void BlackBoardClient::recordingFailed()
{
  publish(BlackBoardEntry::RECORDING_KEY, BlackBoardEntry::FAILED_VALUE, BlackBoardEntry::PURPLE);
}
void BlackBoardClient::streamingFailed()
{
  publish(BlackBoardEntry::STREAMING_KEY, BlackBoardEntry::FAILED_VALUE, BlackBoardEntry::PURPLE);
}

void BlackBoardClient::activatingControl()
{
  publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::freezingControl()
{
  publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::YELLOW);
}

void BlackBoardClient::controlActive()
{
  publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::ACTIVE_VALUE, BlackBoardEntry::GREEN);
}
void BlackBoardClient::controlFrozen()
{
  publish(BlackBoardEntry::CONTROL_KEY, BlackBoardEntry::FROZEN_VALUE, BlackBoardEntry::RED);
}


void BlackBoardClient::setupDebug(const std::string& value)
{
  publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::GREEN);
}
void BlackBoardClient::setupInfo(const std::string& value)
{
  publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::WHITE);
}
void BlackBoardClient::setupWarn(const std::string& value)
{
  publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::setupError(const std::string& value)
{
  publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::RED);
}
void BlackBoardClient::setupFatal(const std::string& value)
{
  publish(BlackBoardEntry::SETUP_KEY, value, BlackBoardEntry::PURPLE);
}

void BlackBoardClient::setup(const std::string& value)
{
  publish(BlackBoardEntry::SETUP_KEY, value);
}

void BlackBoardClient::setRecording(const bool is_recording)
{
  if (is_recording)
  {
    recording();
  }
  else
  {
    recordingStopped();
  }
}
void BlackBoardClient::setStreaming(const bool is_streaming)
{
  if (is_streaming)
  {
    streaming();
  }
  else
  {
    streamingStopped();
  }
}
void BlackBoardClient::setLogging(const bool is_logging)
{
  if (is_logging)
  {
    logging();
  }
  else
  {
    loggingStopped();
  }
}
void BlackBoardClient::setLoggingUnknown()
{
  publish(BlackBoardEntry::LOGGING_KEY, BlackBoardEntry::UNKNOWN_VALUE, BlackBoardEntry::PURPLE);
}

void BlackBoardClient::description(const std::string& value)
{
  publish(BlackBoardEntry::DESCRIPTION_KEY, value);
}
void BlackBoardClient::descriptionRunning(const std::string& value)
{
  publish(BlackBoardEntry::DESCRIPTION_KEY, value, BlackBoardEntry::GREEN);
}
void BlackBoardClient::descriptionFinished(const std::string& value)
{
  publish(BlackBoardEntry::DESCRIPTION_KEY, value, BlackBoardEntry::YELLOW);
}
void BlackBoardClient::descriptionSwapped(const std::string& value)
{
  publish(BlackBoardEntry::DESCRIPTION_KEY, value, BlackBoardEntry::RED);
}
void BlackBoardClient::descriptionFailed(const std::string& value)
{
  publish(BlackBoardEntry::DESCRIPTION_KEY, value, BlackBoardEntry::PURPLE);
}
void BlackBoardClient::descriptionStopped(const std::string& value)
{
  publish(BlackBoardEntry::DESCRIPTION_KEY, value, BlackBoardEntry::RED);
}
void BlackBoardClient::descriptionContinued(const std::string& value)
{
  publish(BlackBoardEntry::DESCRIPTION_KEY, value, BlackBoardEntry::GREEN);
}


void BlackBoardClient::predictionCurrent(const std::string& value)
{
  publish(BlackBoardEntry::PREDICTION_CURRENT_KEY, value);
}

void BlackBoardClient::predictionCurrentProgress(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::PREDICTION_CURRENT_PROGRESS_KEY, ss.str());
}

void BlackBoardClient::predictionCurrentProbability(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::PREDICTION_CURRENT_PROBABILITY_KEY, ss.str());
}

void BlackBoardClient::predictionCurrentCalibrated(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::PREDICTION_CURRENT_CALIBRATED_KEY, ss.str());
}

void BlackBoardClient::predictionSuccessor(const std::string& value)
{
  publish(BlackBoardEntry::PREDICTION_SUCCESSOR_KEY, value);
}

void BlackBoardClient::predictionSuccessorProgress(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::PREDICTION_SUCCESSOR_PROGRESS_KEY, ss.str());
}

void BlackBoardClient::predictionSuccessorProbability(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::PREDICTION_SUCCESSOR_PROBABILITY_KEY, ss.str());
}

void BlackBoardClient::predictionSuccessorCalibrated(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::PREDICTION_SUCCESSOR_CALIBRATED_KEY, ss.str());
}

void BlackBoardClient::predictionFiltered(const std::string& value)
{
  publish(BlackBoardEntry::PREDICTION_FILTERED_KEY, value);
}

void BlackBoardClient::predictionProgressFiltered(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::PREDICTION_PROGRESS_FILTERED_KEY, ss.str());
}

void BlackBoardClient::leftProgress(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::LEFT_PROGRESS_KEY, ss.str());
}
void BlackBoardClient::rightProgress(const float value)
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % value;
  publish(BlackBoardEntry::RIGHT_PROGRESS_KEY, ss.str());
}

}
