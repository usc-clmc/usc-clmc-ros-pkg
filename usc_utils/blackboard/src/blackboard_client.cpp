/*
 * blackboard_client.cpp
 *
 *  Created on: Oct 13, 2013
 *      Author: pastor
 */

#include <blackboard/blackboard_client.h>

namespace blackboard
{

BlackBoardClient::BlackBoardClient() :
    node_handle_("/BlackBoard"), board_("")
{
}

bool BlackBoardClient::initialize(const std::string& board)
{
  board_ = board;
  const int PUBLISHER_BUFFER_SIZE = 10;
  publisher_ = node_handle_.advertise<blackboard::BlackBoardEntry>("entries", PUBLISHER_BUFFER_SIZE);
  return true;
}

void BlackBoardClient::publish(const std::string& key, const std::string& value, const int color)
{
  BlackBoardEntry entry;
  entry.board = board_;
  entry.key = key;
  entry.value = value;
  entry.color = color;
  publisher_.publish(entry);
}

}
