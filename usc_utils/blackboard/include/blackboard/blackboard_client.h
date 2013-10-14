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

  bool initialize(const std::string& board);

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

private:

  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;

  std::string board_;
  void publish(const std::string& key, const std::string& value, const int color);

};

}


#endif /* BLACKBOARD_CLIENT_H_ */
