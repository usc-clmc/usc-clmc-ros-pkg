/*
 * blackboard.cpp
 *
 *  Created on: Oct 13, 2013
 *      Author: pastor
 */

#include <blackboard/blackboard.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

namespace blackboard
{

BlackBoardBase::BlackBoardBase(ros::NodeHandle node_handle)
: node_handle_(node_handle)
{
  text_scale_.x = 1.0;
  text_scale_.y = 1.0;
  ROS_VERIFY(usc_utilities::read(node_handle_, "text_size", text_scale_.z));
  ROS_VERIFY(usc_utilities::read(node_handle_, "line_spacing", line_spacing_));

  marker_colors_.clear();
  std_msgs::ColorRGBA color;
  // BlackBoardEntry::WHITE
  color.r = 0.8;
  color.g = 0.8;
  color.b = 0.8;
  color.a = 0.8;
  marker_colors_.push_back(color);
  // BlackBoardEntry::GREEN
  color.r = 0.0;
  color.g = 0.8;
  color.b = 0.0;
  color.a = 0.8;
  marker_colors_.push_back(color);
  // BlackBoardEntry::YELLOW
  color.r = 0.8;
  color.g = 0.8;
  color.b = 0.0;
  color.a = 0.8;
  marker_colors_.push_back(color);
  // BlackBoardEntry::RED
  color.r = 0.8;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 0.8;
  marker_colors_.push_back(color);
  // BlackBoardEntry::PURPLE
  color.r = 1.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 1.0;
  marker_colors_.push_back(color);

  const int PUBLISHER_BUFFER_SIZE = 1;
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker> ("visualization_marker", PUBLISHER_BUFFER_SIZE);

}


BlackBoardTable::BlackBoardTable(const std::string& board,
                                 const geometry_msgs::Point& position,
                                 ros::NodeHandle node_handle)
  : BlackBoardBase(node_handle), board_(board), position_(position)
{

}

void BlackBoardTable::update(const std::string& key, const std::string& value, const int& color)
{
  bool found = false;
  std::map<std::string, std::string>::iterator it;
  for (it = entries_.begin(); !found && it != entries_.end(); ++it)
  {
    if (it->first.compare(key) == 0)
    {
      ROS_INFO("Updating key >%s< with value >%s<.", key.c_str(), value.c_str());
      it->second = value;
      found = true;
    }
  }
  if (!found)
  {
    ROS_INFO("Adding key >%s< with value >%s<.", key.c_str(), value.c_str());
    entries_.insert(std::pair<std::string, std::string>(key, value));
    ROS_INFO("Number of entries is >%i<.", (int)entries_.size());
    visualization_msgs::Marker marker = getMarker(key, value, color);
    if (markers_.empty())
    {
      marker.pose.position = position_;
    }
    else
    {
      marker.pose.position.z = markers_.back().pose.position.z + line_spacing_;
    }
    markers_.push_back(marker);
  }
}

void BlackBoardTable::publishAll()
{
  for (unsigned int i = 0; i < markers_.size(); ++i)
  {
    publish(i);
  }
}

void BlackBoardTable::publish(const int index)
{
  if (index < 0 || index >= (int)markers_.size())
  {
    ROS_ERROR("Invalid index >%i<. This should never happen. Not publishing.", index);
    return;
  }
  markers_[index].header.stamp = ros::Time::now();
  marker_pub_.publish(markers_[index]);
}

void BlackBoardTable::publish(const std::string& key)
{
  int index = 0;
  std::map<std::string, std::string>::iterator it;
  bool found = false;
  for (it = entries_.begin(); !found && it != entries_.end(); ++it)
  {
    if (it->first.compare(key) == 0)
    {
      found = true;
    }
    else
    {
      index++;
    }
  }

  if (!found)
  {
    ROS_ERROR("Key >%s< not found in >%s<.", key.c_str(), board_.c_str());
    for (it = entries_.begin(); !found && it != entries_.end(); ++it)
    {
      ROS_ERROR("- %s | %s", it->first.c_str(), it->second.c_str());
    }
    return;
  }
  publish(index);
}

BlackBoard::BlackBoard(ros::NodeHandle node_handle)
: node_handle_(node_handle)
{
  // do this last
  const unsigned int MESSAGE_SUBSCRIBER_BUFFER_SIZE = 10;
  subscriber_ = node_handle_.subscribe("entries", MESSAGE_SUBSCRIBER_BUFFER_SIZE, &BlackBoard::blackboard, this);
}

visualization_msgs::Marker BlackBoardTable::getMarker(const std::string& key,
                                                      const std::string& value,
                                                      const int& color)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/BASE";
  marker.id = markers_.size();
  marker.ns = board_ + "_" + key;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.pose.position = position_;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale = text_scale_;
  marker.color = marker_colors_[color];
  marker.text = key + ": " + value;
  return marker;
}

void BlackBoard::blackboard(const BlackBoardEntry::ConstPtr blackboard_entry)
{
  int color = blackboard_entry->color;
  if (color < 0 && color >= BlackBoardEntry::NUM_COLORS)
  {
    ROS_ERROR("Invalid color >%i<. Setting color to red.", color);
    color = (int)BlackBoardEntry::RED;
  }

  bool found = false;
  for (unsigned int i = 0; !found && i < blackboard_tables_.size(); ++i)
  {
    if (blackboard_tables_[i].is(blackboard_entry->board))
    {
      blackboard_tables_[i].update(blackboard_entry->key, blackboard_entry->value, color);
      blackboard_tables_[i].publish(blackboard_entry->key);
      found = true;
    }
  }
  if (!found)
  {
    geometry_msgs::Point position;
    if(blackboard_entry->board.empty())
    {
      ROS_ERROR("Empty board name received. Screw that.");
      return;
    }
    std::string param_name = blackboard_entry->board + "_blackboard_position";
    ROS_INFO("Reading >%s<.", param_name.c_str());
    ROS_VERIFY(usc_utilities::read(node_handle_, param_name, position));
    BlackBoardTable blackboard_table(blackboard_entry->board, position, node_handle_);
    blackboard_table.update(blackboard_entry->key, blackboard_entry->value, color);
    blackboard_tables_.push_back(blackboard_table);
    blackboard_tables_.back().publish(blackboard_entry->key);
  }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  ros::NodeHandle node_handle("~");
  blackboard::BlackBoard blackboard(node_handle);
  blackboard.run();
  return 0;
}
