/*
 * rviz_publisher.cpp
 *
 *  Created on: Jan 7, 2011
 *      Author: kalakris
 */

#include <usc_utilities/rviz_publisher.h>

namespace usc_utilities
{

RvizPublisher::RvizPublisher(const std::string& topic_name)
{
  pub_ = node_handle_.advertise<visualization_msgs::Marker>(topic_name, 100, true);
}

RvizPublisher::~RvizPublisher()
{
}

void RvizPublisher::publish(const visualization_msgs::Marker& marker)
{
  pub_.publish(marker);
}


}
