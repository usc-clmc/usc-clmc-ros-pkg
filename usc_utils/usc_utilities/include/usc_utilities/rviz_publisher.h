/*
 * rviz_publisher.h
 *
 *  Created on: Jan 7, 2011
 *      Author: kalakris
 */

#ifndef RVIZ_PUBLISHER_H_
#define RVIZ_PUBLISHER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace usc_utilities
{

class RvizPublisher
{
public:
  RvizPublisher(const std::string& topic_name);
  virtual ~RvizPublisher();

  void publish(const visualization_msgs::Marker& marker);

private:
  ros::NodeHandle node_handle_;
  ros::Publisher pub_;
};

}

#endif /* RVIZ_PUBLISHER_H_ */
