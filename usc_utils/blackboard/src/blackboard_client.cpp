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
  publisher_.publish(entry);
  if (single_threaded_)
  {
    // ROS_DEBUG("Spinning.");
    ros::spinOnce();
  }
}

void BlackBoardClient::reset()
{
  info(BlackBoardEntry::LOGGING_KEY, "");
  info(BlackBoardEntry::STREAMING_KEY, "");
  info(BlackBoardEntry::RECORDING_KEY, "");
  info(BlackBoardEntry::SETUP_KEY, "");
}


RightBlackBoardClient::RightBlackBoardClient(const bool single_threaded)
{
  ROS_VERIFY(initialize("right", single_threaded));
}

LeftBlackBoardClient::LeftBlackBoardClient(const bool single_threaded)
{
  ROS_VERIFY(initialize("left", single_threaded));
}

}
