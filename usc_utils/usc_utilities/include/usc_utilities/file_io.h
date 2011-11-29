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
#include <sensor_msgs/JointState.h>

// local includes

namespace usc_utilities
{

// default template parameters
template<class MessageType = sensor_msgs::JointState> class FileIO;

template<class MessageType>
  class FileIO
  {

  public:


    /*!
     * @param msgs
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool readFromBagFile(std::vector<MessageType>& msgs,
                                const std::string& topic_name,
                                const std::string& abs_bag_file_name,
                                bool verbose = true);

    /*!
     * @param msg
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool readFromBagFile(MessageType& msg,
                                const std::string& topic_name,
                                const std::string& abs_bag_file_name,
                                bool verbose = true);

    /*!
     * @param msgs
     * @param times
     * @param topic_name
     * @param abs_bag_file_name
     * @param mode
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                             const std::vector<ros::Time>& times,
                                             const std::string& topic_name,
                                             const std::string& abs_bag_file_name,
                                             rosbag::BagMode mode,
                                             bool verbose = true);

    /*!
     * @param msgs
     * @param topic_name
     * @param abs_bag_file_name
     * @param mode
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool writeToBagFile(const std::vector<MessageType>& msgs,
                               const std::string& topic_name,
                               const std::string& abs_bag_file_name,
                               rosbag::BagMode mode = rosbag::bagmode::Write,
                               bool verbose = true);

    /*!
     * @param msgs
     * @param topic_name
     * @param abs_bag_file_name
     * @param mode
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                             const std::string& topic_name,
                                             const std::string& abs_bag_file_name,
                                             rosbag::BagMode mode = rosbag::bagmode::Write,
                                             bool verbose = true);
    /*!
     * @param msgs
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                             const std::string& topic_name,
                                             const std::string& abs_bag_file_name,
                                             bool verbose);

    /*!
     * @param msg
     * @param topic_name
     * @param abs_bag_file_name
     * @param mode
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool writeToBagFile(const MessageType& msg,
                               const std::string& topic_name,
                               const std::string& abs_bag_file_name,
                               rosbag::BagMode mode = rosbag::bagmode::Write,
                               bool verbose = true);

    /*!
     * @param msg
     * @param topic_name
     * @param abs_bag_file_name
     * @param mode
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool writeToBagFileWithTimeStamp(const MessageType& msg,
                                            const ros::Time& time_stamp,
                                            const std::string& topic_name,
                                            const std::string& abs_bag_file_name,
                                            rosbag::BagMode mode = rosbag::bagmode::Write,
                                            bool verbose = true);

    /*!
     * @param msgs
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool appendToBagFile(const std::vector<MessageType>& msgs,
                                const std::string& topic_name,
                                const std::string& abs_bag_file_name,
                                bool verbose = true);

    /*!
     * @param msgs
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool appendToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                              const std::string& topic_name,
                                              const std::string& abs_bag_file_name,
                                              bool verbose = true);

    /*!
     * @param msgs
     * @param times
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool appendToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                              const std::vector<ros::Time>& times,
                                              const std::string& topic_name,
                                              const std::string& abs_bag_file_name,
                                              bool verbose = true);

    /*!
     * @param msg
     * @param time
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool appendToBagFileWithTimeStamp(const MessageType& msg,
                                             const ros::Time time,
                                             const std::string& topic_name,
                                             const std::string& abs_bag_file_name,
                                             bool verbose = true);

    /*!
     * @param msg
     * @param topic_name
     * @param abs_bag_file_name
     * @param verbose
     * @return True on success, otherwise False
     */
    static bool appendToBagFile(const MessageType& msg,
                                const std::string& topic_name,
                                const std::string& abs_bag_file_name,
                                bool verbose = true);

    /*!
     * @param topic_name
     * @param abs_bag_file_name
     * @return True on success, otherwise False
     */
    static bool containsMsg(const std::string& topic_name,
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
                                            const std::string& abs_bag_file_name,
                                            bool verbose)
  {
    ROS_INFO_COND(verbose, "Reading topic named >%s< from bag file >%s<", topic_name.c_str(), abs_bag_file_name.c_str());
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
        else
        {
          ROS_ERROR("Null message read from file >%s< on topic >%s<.", abs_bag_file_name.c_str(), topic_name.c_str());
          return false;
        }
      }
      bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
      ROS_ERROR("Problem when reading from bag file >%s< : %s.", abs_bag_file_name.c_str(), ex.what());
      return false;
    }
    ROS_INFO_COND(verbose, "Read >%i< messages.", (int)msgs.size());
    ROS_WARN_COND(msgs.empty(), "No messages read from file >%s< on topic >%s<.", abs_bag_file_name.c_str(), topic_name.c_str());
    return true;
  }

template<class MessageType>
  bool FileIO<MessageType>::readFromBagFile(MessageType& msg,
                                            const std::string& topic_name,
                                            const std::string& abs_bag_file_name,
                                            bool verbose)
  {
    std::vector<MessageType> msgs;
    if (!readFromBagFile(msgs, topic_name, abs_bag_file_name, verbose))
    {
      ROS_ERROR("Could not read messages from topic >%s< from bag file >%s<.", topic_name.c_str(), abs_bag_file_name.c_str());
      return false;
    }
    if (msgs.size() != 1)
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
                                           const std::string& abs_bag_file_name,
                                           rosbag::BagMode mode,
                                           bool verbose)
  {
    std::vector<ros::Time> times;
    for (int i = 0; i < (int)msgs.size(); ++i)
    {
      times.push_back(ros::Time::now());
    }
    return writeToBagFileWithTimeStamps(msgs, times, topic_name, abs_bag_file_name, mode, verbose);
  }

template<class MessageType>
  bool FileIO<MessageType>::writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                                         const std::vector<ros::Time>& times,
                                                         const std::string& topic_name,
                                                         const std::string& abs_bag_file_name,
                                                         rosbag::BagMode mode,
                                                         bool verbose)
  {
    ROS_ASSERT_MSG(!msgs.empty(), "Messages are empty. Cannot write anything to file >%s<.", abs_bag_file_name.c_str());
    ROS_INFO_COND(verbose, "Writing to bag file: %s", abs_bag_file_name.c_str());
    // ROS_WARN_COND(containsMsg(topic_name, abs_bag_file_name), "Topic named >%s< already contained in the bag file >%s<.", topic_name.c_str(), abs_bag_file_name.c_str());
    try
    {
      rosbag::Bag bag(abs_bag_file_name, mode);
      ROS_ASSERT(msgs.size() == times.size());
      for (int i = 0; i < (int)msgs.size(); ++i)
      {
        bag.write(topic_name, times[i], msgs[i]);
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
  bool FileIO<MessageType>::containsMsg(const std::string& topic_name,
                                        const std::string& abs_bag_file_name)
  {
    bool topic_contained = false;
    try
    {
      rosbag::Bag bag(abs_bag_file_name, rosbag::bagmode::Read);
      rosbag::View view(bag, rosbag::TopicQuery(topic_name));
      for (rosbag::View::iterator it = view.begin(); it != view.end() && !topic_contained; ++it)
      {
        if (it->getTopic().compare(topic_name) == 0)
        {
          topic_contained = true;
        }
      }
      bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
      ROS_ERROR("Problem when checking bag file named >%s< : %s.", abs_bag_file_name.c_str(), ex.what());
      return false;
    }
    return topic_contained;
  }

template<class MessageType>
  bool FileIO<MessageType>::writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                                         const std::string& topic_name,
                                                         const std::string& abs_bag_file_name,
                                                         rosbag::BagMode mode,
                                                         bool verbose)
  {
    std::vector<ros::Time> times;
    for (int i = 0; i < (int)msgs.size(); ++i)
    {
      times.push_back(msgs[i].header.stamp);
    }
    return writeToBagFileWithTimeStamps(msgs, times, topic_name, abs_bag_file_name, mode, verbose);
  }

template<class MessageType>
  bool FileIO<MessageType>::writeToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                                         const std::string& topic_name,
                                                         const std::string& abs_bag_file_name,
                                                         bool verbose)
  {
    return writeToBagFileWithTimeStamps(msgs, topic_name, abs_bag_file_name, rosbag::bagmode::Write, verbose);
  }

template<class MessageType>
  bool FileIO<MessageType>::writeToBagFile(const MessageType& msg,
                                           const std::string& topic_name,
                                           const std::string& abs_bag_file_name,
                                           rosbag::BagMode mode,
                                           bool verbose)
  {
    std::vector<MessageType> msgs;
    msgs.push_back(msg);
    return writeToBagFile(msgs, topic_name, abs_bag_file_name, mode, verbose);
  }

template<class MessageType>
  bool FileIO<MessageType>::writeToBagFileWithTimeStamp(const MessageType& msg,
                                                        const ros::Time& time_stamp,
                                                        const std::string& topic_name,
                                                        const std::string& abs_bag_file_name,
                                                        rosbag::BagMode mode,
                                                        bool verbose)
  {
    std::vector<MessageType> msgs;
    msgs.push_back(msg);
    std::vector<ros::Time> time_stamps;
    time_stamps.push_back(time_stamp);
    return writeToBagFileWithTimeStamps(msgs, time_stamps, topic_name, abs_bag_file_name, mode, verbose);
  }

template<class MessageType>
  bool FileIO<MessageType>::appendToBagFile(const std::vector<MessageType>& msgs,
                                            const std::string& topic_name,
                                            const std::string& abs_bag_file_name,
                                            bool verbose)
  {
    return writeToBagFile(msgs, topic_name, abs_bag_file_name, rosbag::bagmode::Append, verbose);
  }

template<class MessageType>
  bool FileIO<MessageType>::appendToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                                          const std::string& topic_name,
                                                          const std::string& abs_bag_file_name,
                                                          bool verbose)
  {
    return writeToBagFileWithTimeStamps(msgs, topic_name, abs_bag_file_name, rosbag::bagmode::Append, verbose);
  }

template<class MessageType>
  bool FileIO<MessageType>::appendToBagFileWithTimeStamps(const std::vector<MessageType>& msgs,
                                                          const std::vector<ros::Time>& times,
                                                          const std::string& topic_name,
                                                          const std::string& abs_bag_file_name,
                                                          bool verbose)
{
  return writeToBagFileWithTimeStamps(msgs, times, topic_name, abs_bag_file_name, rosbag::bagmode::Append, verbose);
}

template<class MessageType>
  bool FileIO<MessageType>::appendToBagFileWithTimeStamp(const MessageType& msg,
                                                         const ros::Time time,
                                                         const std::string& topic_name,
                                                         const std::string& abs_bag_file_name,
                                                         bool verbose)
{
  std::vector<MessageType> msgs;
  msgs.push_back(msg);
  std::vector<ros::Time> times;
  times.push_back(time);
  return writeToBagFileWithTimeStamps(msgs, times, topic_name, abs_bag_file_name, rosbag::bagmode::Append, verbose);
}

template<class MessageType>
  bool FileIO<MessageType>::appendToBagFile(const MessageType& msg,
                                            const std::string& topic_name,
                                            const std::string& abs_bag_file_name,
                                            bool verbose)
  {
    return writeToBagFile(msg, topic_name, abs_bag_file_name, rosbag::bagmode::Append, verbose);
  }

}

#endif /* UTILITIES_FILE_IO_H_ */
