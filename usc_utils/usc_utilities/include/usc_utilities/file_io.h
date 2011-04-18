/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		file_io.h

  \author	Peter Pastor
  \date		Jan 5, 2011

 *********************************************************************/

#ifndef UTILITIES_FILE_IO_H_
#define UTILITIES_FILE_IO_H_

// system includes
#include <vector>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// local includes

namespace usc_utilities
{

template<class MessageType>
  class FileIO
  {

  public:

    /*!
     * @param msgs
     * @param abs_bag_file_name
     * @return
     */
    static bool readFromBagFile(std::vector<MessageType>& msgs,
                                const std::string& topic_name,
                                const std::string& abs_bag_file_name);

    static bool readFromBagFile(MessageType& msg,
                                const std::string& topic_name,
                                const std::string& abs_bag_file_name);

    static bool writeToBagFile(const std::vector<MessageType>& msgs,
                               const std::string& topic_name,
                               const std::string& abs_bag_file_name);

    static bool writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                               const std::string& topic_name,
                               const std::string& abs_bag_file_name);

    static bool writeToBagFile(const MessageType& msg,
                               const std::string& topic_name,
                               const std::string& abs_bag_file_name);

  private:

    /*! Constructor
     */
    FileIO() {};

    /*! Destructor
     */
    virtual ~FileIO() {};

  };

template<class MessageType>
  bool FileIO<MessageType>::readFromBagFile(std::vector<MessageType>& msgs,
                                            const std::string& topic_name,
                                            const std::string& abs_bag_file_name)
  {
    ROS_INFO("Reading from bag file: %s", abs_bag_file_name.c_str());
    try
    {
      rosbag::Bag bag(abs_bag_file_name, rosbag::bagmode::Read);
      rosbag::View view(bag, rosbag::TopicQuery(topic_name));

      msgs.clear();
      BOOST_FOREACH(rosbag::MessageInstance const msg_instance, view)
      {
        typename MessageType::ConstPtr msg = msg_instance.instantiate<MessageType> ();
        if (msg != NULL)
        {
          msgs.push_back(*msg);
        }
      }
      bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
      ROS_ERROR("Problem when reading from bag file >%s< : %s.", abs_bag_file_name.c_str(), ex.what());
      return false;
    }
    return true;
  }

template<class MessageType>
  bool FileIO<MessageType>::readFromBagFile(MessageType& msg,
                                            const std::string& topic_name,
                                            const std::string& abs_bag_file_name)
  {
    std::vector<MessageType> msgs;
    if(!readFromBagFile(msgs, topic_name, abs_bag_file_name))
    {
      ROS_ERROR("Could not read messages from topic >%s< from bag file >%s<.", topic_name.c_str(), abs_bag_file_name.c_str());
      return false;
    }
    if(msgs.size() != 1)
    {
      ROS_ERROR("There are >%i< messages contained under the topic named >%s< in the bag file >%s<, cannot read single message.", (int)msgs.size(), topic_name.c_str(), abs_bag_file_name.c_str());
      return false;
    }
    msg = msgs[0];
    return true;
  }

template<class MessageType>
  bool FileIO<MessageType>::writeToBagFile(const std::vector<MessageType>& msgs,
                                           const std::string& topic_name,
                                           const std::string& abs_bag_file_name)
  {
    ROS_INFO("Writing to bag file: %s", abs_bag_file_name.c_str());
    try
    {
      rosbag::Bag bag;
      bag.open(abs_bag_file_name, rosbag::bagmode::Write);
      for (int i = 0; i < (int)msgs.size(); ++i)
      {
        bag.write(topic_name, ros::Time::now(), msgs[i]);
      }
      bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
      ROS_ERROR("Problem when writing to bag file named >%s< : %s.", abs_bag_file_name.c_str(), ex.what());
      return false;
    }
    return true;
  }


template<class MessageType>
  bool FileIO<MessageType>::writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                           const std::string& topic_name,
                                           const std::string& abs_bag_file_name)
  {
    ROS_INFO("Writing to bag file: %s", abs_bag_file_name.c_str());
    try
    {
      rosbag::Bag bag;
      bag.open(abs_bag_file_name, rosbag::bagmode::Write);
      for (int i = 0; i < (int)msgs.size(); ++i)
      {
        bag.write(topic_name, msgs[i].header.stamp, msgs[i]);
      }
      bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
      ROS_ERROR("Problem when writing to bag file named >%s< : %s.", abs_bag_file_name.c_str(), ex.what());
      return false;
    }
    return true;
  }

template<class MessageType>
  bool FileIO<MessageType>::writeToBagFile(const MessageType& msg,
                                           const std::string& topic_name,
                                           const std::string& abs_bag_file_name)
  {
    std::vector<MessageType> msgs;
    msgs.push_back(msg);
    return writeToBagFile(msgs, topic_name, abs_bag_file_name);
  }

}

#endif /* UTILITIES_FILE_IO_H_ */
