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

BlackBoardBase::BlackBoardBase(ros::NodeHandle& node_handle,
                               ros::Publisher& marker_publisher)
: node_handle_(node_handle), marker_publisher_(marker_publisher)
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
  // BlackBoardEntry::BLUE
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 1.0;
  marker_colors_.push_back(color);
}

BlackBoardTable::BlackBoardTable(const std::string& board,
                                 const geometry_msgs::Point& position,
                                 ros::NodeHandle& node_handle,
                                 ros::Publisher& marker_publisher)
  : BlackBoardBase(node_handle, marker_publisher), board_(board), position_(position)
{
}

void BlackBoardTable::update(const std::string& key, const std::string& value, const int& color)
{
  it_ = entries_.find(key);
  if (it_ != entries_.end())
  {
    ROS_DEBUG("Updating key >%s< with value >%s<.", key.c_str(), value.c_str());
    it_->second.first = value;
    it_->second.second.text = key + ": " + value;
  }
  else
  {
    visualization_msgs::Marker marker = getMarker(key, value, color);
    position_.z += line_spacing_;
    ROS_DEBUG("Adding key >%s< with value >%s< at >%.2f<.", key.c_str(), value.c_str(), marker.pose.position.z);
    std::pair <std::string, visualization_msgs::Marker> values(value, marker);
    entries_.insert(std::pair<std::string, std::pair<std::string, visualization_msgs::Marker> >(key, values));
  }
}

void BlackBoardTable::publishAll()
{
  for (it_ = entries_.begin(); it_ != entries_.end(); ++it_)
  {
    publish(it_->second.second);
  }
}

void BlackBoardTable::publish(visualization_msgs::Marker& marker)
{
  marker.header.stamp = ros::Time::now();
  if (marker_publisher_.getNumSubscribers() > 0)
    marker_publisher_.publish(marker);
  else
    ROS_WARN("No subscribers listening. So not bothering publishing.");
}

void BlackBoardTable::publish(const std::string& key, const int color, const bool update_color)
{
  it_ = entries_.find(key);
  if (it_ != entries_.end())
  {
    if (update_color)
    {
      it_->second.second.color = marker_colors_[color];
    }
    publish(it_->second.second);
  }
  else
  {
    ROS_ERROR("Key >%s< not found in >%s<.", key.c_str(), board_.c_str());
    for (it_ = entries_.begin(); it_ != entries_.end(); ++it_)
    {
      ROS_ERROR("- %s | %s", it_->first.c_str(), it_->second.first.c_str());
    }
    return;
  }
}

BlackBoard::BlackBoard(ros::NodeHandle node_handle)
: node_handle_(node_handle)
{
  const int PUBLISHER_BUFFER_SIZE = 30;
  marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker> ("visualization_marker", PUBLISHER_BUFFER_SIZE);

  // do this last
  const unsigned int MESSAGE_SUBSCRIBER_BUFFER_SIZE = 10;
  subscriber_ = node_handle_.subscribe("entries", MESSAGE_SUBSCRIBER_BUFFER_SIZE, &BlackBoard::blackboard, this);
}

visualization_msgs::Marker BlackBoardTable::getMarker(const std::string& key,
                                                      const std::string& value,
                                                      const int color)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/BASE";
  marker.id = entries_.size();
  marker.ns = board_ + "_" + key;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.pose.position = position_;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale = text_scale_;
  int local_color = color;
  if (local_color < 0 || local_color >= BlackBoardEntry::NUM_COLORS)
  {
    local_color = BlackBoardEntry::PURPLE;
    ROS_ERROR("Invalid color index provided >%i<. Setting color to >%i<.", color, local_color);
  }
  marker.color = marker_colors_[local_color];
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
      if (blackboard_entry->action != BlackBoardEntry::CHANGE_COLOR)
      {
        blackboard_tables_[i].update(blackboard_entry->key, blackboard_entry->value, color);
      }
      const bool change_color = blackboard_entry->action != BlackBoardEntry::UPDATE_BUT_KEEP_COLOR;
      blackboard_tables_[i].publish(blackboard_entry->key, color, change_color);
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
    // ROS_INFO("Reading >%s<.", param_name.c_str());
    ROS_VERIFY(usc_utilities::read(node_handle_, param_name, position));
    BlackBoardTable blackboard_table(blackboard_entry->board, position, node_handle_, marker_publisher_);
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
  ros::MultiThreadedSpinner mts;
  mts.spin();
  return 0;
}
