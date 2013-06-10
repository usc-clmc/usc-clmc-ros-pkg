/*
 * msg_header_utility.h
 *
 *  Created on: Jun 7, 2013
 *      Author: pastor
 */

#ifndef MSG_HEADER_UTILITY_H_
#define MSG_HEADER_UTILITY_H_

#include <ros/ros.h>
#include <usc_utilities/assert.h>

namespace task_recorder2
{

template<class MessageType>
  class HeaderUtility
  {
  public:
    HeaderUtility() {};
    virtual ~HeaderUtility() {};
    virtual std_msgs::Header header(const MessageType& message) const = 0;
};

template<class MessageType>
  class TFHeaderExtractor : public HeaderUtility<MessageType>
  {
  public:
    std_msgs::Header header(const MessageType& message) const;
  };

template<class MessageType>
std_msgs::Header TFHeaderExtractor<MessageType>::header(const MessageType& message) const
{
  ROS_ASSERT(!message.transforms.empty());
  return message.transforms[0].header;
}

}

#endif /* MSG_HEADER_UTILITY_H_ */
