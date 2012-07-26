/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_usc_utilities.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 5, 2011

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_IO_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_IO_H_

// system includes
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <usc_utilities/file_io.h>

// local includes

namespace dmp
{

template<class DMPType, class MessageType>
class DynamicMovementPrimitiveIO
{

public:

  /*!
   * @param msg
   * @param abs_bag_file_name
   * @param verbose
   * @return True if successful, otherwise False
   */
  static bool writeToDisc(const MessageType& msg,
                          const std::string& abs_bag_file_name,
                          bool verbose = true);

  /*!
   * @param dmp
   * @param abs_bag_file_name
   * @param verbose
   * @return True if successful, otherwise False
   */
  static bool writeToDisc(const typename DMPType::DMPPtr& dmp,
                          const std::string& abs_bag_file_name,
                          bool verbose = true);

  /*!
   * @param dmp
   * @param abs_bag_file_name
   * @param must_contain_dmp_msg
   * @return True if successful, otherwise False
   */
  static bool readFromDisc(typename DMPType::DMPPtr& dmp,
                           const std::string& abs_bag_file_name,
                           bool must_contain_dmp_msg = true);


private:

  /*! Constructor
   */
  DynamicMovementPrimitiveIO() {};

  /*! Destructor
   */
  virtual ~DynamicMovementPrimitiveIO() {};

};

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveIO<DMPType, MessageType>::writeToDisc(const MessageType& msg,
                                                                     const std::string& abs_bag_file_name,
                                                                     bool verbose)
  {
    return usc_utilities::FileIO<MessageType>::writeToBagFile(msg, DMPType::getVersionString(), abs_bag_file_name, rosbag::bagmode::Write, verbose);
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveIO<DMPType, MessageType>::writeToDisc(const typename DMPType::DMPPtr& dmp,
                                                                     const std::string& abs_bag_file_name,
                                                                     bool verbose)
  {
    MessageType message;
    if(!dmp->writeToMessage(message))
    {
      return false;
    }
    return usc_utilities::FileIO<MessageType>::writeToBagFile(message, DMPType::getVersionString(), abs_bag_file_name, rosbag::bagmode::Write, verbose);
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveIO<DMPType, MessageType>::readFromDisc(typename DMPType::DMPPtr& dmp,
                                                                      const std::string& abs_bag_file_name,
                                                                      bool must_contain_dmp_msg)
  {
    ROS_DEBUG("Reading from bag file: %s", abs_bag_file_name.c_str());
    try
    {
      rosbag::Bag bag(abs_bag_file_name, rosbag::bagmode::Read);
      std::string topic_name = DMPType::getVersionString();
      rosbag::View view(bag, rosbag::TopicQuery(topic_name));

      int num_msgs = 0;
      BOOST_FOREACH(rosbag::MessageInstance const msg_instance, view)
      {
        if(num_msgs > 0)
        {
          ROS_ERROR("Bag file contains more than 1 message on topic >%s<.", topic_name.c_str());
          bag.close();
          return false;
        }
        typename MessageType::ConstPtr msg = msg_instance.instantiate<MessageType> ();
        if (msg != NULL)
        {
           num_msgs++;
           ROS_VERIFY(DMPType::createFromMessage(dmp, *msg));
        }
      }
      if(must_contain_dmp_msg && num_msgs != 1)
      {
        ROS_ERROR("Could not read msg of topic >%s< bag file >%s<.", topic_name.c_str(), abs_bag_file_name.c_str());
        bag.close();
        return false;
      }
      bag.close();
    }
    catch (rosbag::BagIOException& ex)
    {
      ROS_ERROR("Problem when reading from bag file >%s< : %s.", abs_bag_file_name.c_str(), ex.what());
      return false;
    }
    return true;
  }

}


#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_IO_H_ */
