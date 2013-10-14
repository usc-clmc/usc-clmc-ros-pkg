/*
 * bag_file_recorder_client.h
 *
 *  Created on: Jun 10, 2013
 *      Author: pastor
 */

#ifndef BAG_FILE_RECORDER_CLIENT_H_
#define BAG_FILE_RECORDER_CLIENT_H_

#include <usc_utilities/assert.h>
#include <boost/shared_ptr.hpp>

#include <task_recorder2_bags/bag_file_recorder.h>
#include <task_recorder2_bags/msg_header_utility.h>

namespace task_recorder2_bags
{

template<class MessageType, class MessageTypeStamped, class MessageUtility>
  class BagFileRecorderClient : public BagFileRecorder<MessageType, MessageTypeStamped>
  {
  public:
    BagFileRecorderClient(const std::string& topic_name);
    virtual ~BagFileRecorderClient() {};
  };

template<class MessageType, class MessageTypeStamped, class MessageUtility>
  BagFileRecorderClient<MessageType, MessageTypeStamped, MessageUtility>::BagFileRecorderClient(const std::string& topic_name)
{
  boost::shared_ptr<MessageUtility> header_utility(new MessageUtility());
  ROS_ASSERT(initialize(ros::NodeHandle("/TaskRecorderManager"), topic_name, header_utility));
}

}

#endif /* BAG_FILE_RECORDER_CLIENT_H_ */
