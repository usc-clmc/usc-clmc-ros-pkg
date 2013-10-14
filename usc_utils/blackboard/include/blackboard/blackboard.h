/*
 * blackboard.h
 *
 *  Created on: Oct 13, 2013
 *      Author: pastor
 */

#ifndef BLACKBOARD_H_
#define BLACKBOARD_H_

#include <ros/ros.h>

#include <blackboard/BlackBoardEntry.h>

#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>

namespace blackboard
{

class BlackBoardBase
{

public:

  BlackBoardBase(ros::NodeHandle node_handle);
  virtual ~BlackBoardBase() {};

protected:
  std::vector<std_msgs::ColorRGBA> marker_colors_;

  ros::NodeHandle node_handle_;
  ros::Publisher marker_pub_;

  geometry_msgs::Vector3 text_scale_;
  float line_spacing_;

private:

};

class BlackBoardTable : public BlackBoardBase
{

public:
  BlackBoardTable(const std::string& board,
                  const geometry_msgs::Point& position,
                  ros::NodeHandle node_handle);
  virtual ~BlackBoardTable() {};

  /*! Updates the display based on key, value, and color
   * @param key
   * @param value
   * @param color
   */
  void update(const std::string& key, const std::string& value, const int& color);

  /*!
   */
  bool is(const std::string& board) const
  {
    return (board.compare(board_) == 0);
  }

  void publish(const std::string& key, const int color = 0, const bool update_color = false);
  void publishAll();

private:

  std::string board_;
  std::map<std::string, std::pair<std::string, visualization_msgs::Marker> > entries_;
  std::map<std::string, std::pair<std::string, visualization_msgs::Marker> >::iterator it_;
  visualization_msgs::Marker getMarker(const std::string& key,
                                       const std::string& value,
                                       const int color);

  void addMarker(visualization_msgs::Marker& marker);
  geometry_msgs::Point position_;
  void publish(visualization_msgs::Marker& marker);
};

class BlackBoard
{

public:

  BlackBoard(ros::NodeHandle node_handle);
  virtual ~BlackBoard() {};

  void run()
  {
    ros::spin();
  }

  void blackboard(const BlackBoardEntry::ConstPtr blackboard_entry);

private:

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  std::vector<BlackBoardTable> blackboard_tables_;

};

}


#endif /* BLACKBOARD_H_ */
