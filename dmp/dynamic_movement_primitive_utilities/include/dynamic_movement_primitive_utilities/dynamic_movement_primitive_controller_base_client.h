/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		dynamic_movement_primitive_controller_base_client.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Jan 24, 2011

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_BASE_CLIENT_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_BASE_CLIENT_H_

// system includes
#include <string>
#include <vector>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <usc_utilities/param_server.h>
#include <robot_info/robot_info.h>
#include <dynamic_movement_primitive/ControllerStatusMsg.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>

// local includes

namespace dmp_utilities
{

template<class DMPType, class MessageType>
  class DynamicMovementPrimitiveControllerBaseClient
  {

  public:

    friend class DynamicMovementPrimitiveControllerClient;

    /*!
     */
    DynamicMovementPrimitiveControllerBaseClient() {};
    ~DynamicMovementPrimitiveControllerBaseClient() {};

    /**
     * Sets this client to single-threaded mode. This means that if the waitForCompletion()
     * function is called, it will spin when in single-threaded mode to listen for status messages,
     * otherwise callbacks are expected to be processed separately.
     */
    void setSingleThreadedMode(bool single_threaded_mode = true);

    /**
     * Send a dmp message to the appropriate dmp controller.
     * Optionally wait for success.
     *
     * @param dmp_msg
     * @param sequence_number
     * @param wait_for_success
     * @return true if command was published successfully, false if not
     */
    bool sendCommand(MessageType& dmp_msg,
                     const int sequence_number,
                     bool wait_for_success = true);

    /**
     * Checks if the controller is still executing a DMP
     *
     * @return True if controller is active, False if not
     */
    bool isActive();

    /**
     * Gets the status of the last executed DMP
     *
     * @return False if no DMP has ever been executed through this client
     */
    bool getControllerStatus(dynamic_movement_primitive::ControllerStatusMsg& dmp_status);

    /**
     * Wait for completion of the last sent command.
     *
     * @return true on success, false if the command was preempted or timed out
     */
    bool waitForCompletion();

  protected:

    /*!
     * @param topic_name
     * @return True on success, otherwise False
     */
    bool initialize(const std::string& topic_name);

    /*!
     */
    bool single_threaded_mode_;

  private:

    /*!
     */
    ros::NodeHandle node_handle_;
    ros::Publisher command_publisher_;
    ros::Subscriber status_subcriber_;

    /*!
     */
    dynamic_movement_primitive::ControllerStatusMsg last_dmp_status_;
    boost::mutex dmp_status_mutex_;
    boost::condition_variable dmp_status_condition_;

    /*!
     */
    int sequence_number_;
    enum
    {
      IDLE,
      ACTIVE,
    } client_state_;

    void statusCallback(const dynamic_movement_primitive::ControllerStatusMsg::ConstPtr& msg);

  };

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::initialize(const std::string& topic_name)
  {
    std::string name = topic_name;
    usc_utilities::appendTrailingSlash(name);

    const int MESSAGE_BUFFER_SIZE = 100;

    // advertise dmp commands:
    std::string command_topic_name = name + "command";
    ROS_DEBUG("Creating publisher to topic >%s<.",command_topic_name.c_str());
    command_publisher_ = node_handle_.advertise<MessageType> (command_topic_name, MESSAGE_BUFFER_SIZE);

    // subscribe to dmp status:
    std::string status_topic_name = name + "status";
    ROS_DEBUG("Creating subscriber to topic >%s<.",status_topic_name.c_str());
    status_subcriber_ = node_handle_.subscribe(status_topic_name, MESSAGE_BUFFER_SIZE,
                                               &DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::statusCallback, this);

    sequence_number_ = -1;
    client_state_ = IDLE;
    return true;
  }

template<class DMPType, class MessageType>
  void DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::statusCallback(const dynamic_movement_primitive::ControllerStatusMsg::ConstPtr& msg)
  {
    ROS_INFO("Received message with sequence number >%i<...", msg->seq);

    boost::mutex::scoped_lock lock(dmp_status_mutex_);
    if(msg->seq == sequence_number_)
    {
      if(msg->status == dynamic_movement_primitive::ControllerStatusMsg::STARTED)
      {
        ROS_INFO("DMP has started...");
        // store last message
        last_dmp_status_ = *msg;
        client_state_ = IDLE;
        dmp_status_condition_.notify_all();
      }
      ROS_INFO_COND(msg->status == dynamic_movement_primitive::ControllerStatusMsg::FINISHED, "DMP has finished...");
      ROS_INFO_COND(msg->status == dynamic_movement_primitive::ControllerStatusMsg::SWAPPED, "DMP has beed swapped...");
      ROS_ERROR_COND(msg->status == dynamic_movement_primitive::ControllerStatusMsg::FAILED, "DMP has failed...");
    }
    else
    {
      if(msg->status == dynamic_movement_primitive::ControllerStatusMsg::SWAPPED)
      {
        ROS_INFO("Previous DMP >%i< has been swapped.", sequence_number_);
      }
      ROS_WARN("Sequence number received >%i<. However, waiting for sequence number >%i<. Ignoring...", msg->seq, sequence_number_);
    }
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::isActive()
  {
  boost::mutex::scoped_lock lock(dmp_status_mutex_);
    return client_state_ != IDLE;
  }

template<class DMPType, class MessageType>
  void DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::setSingleThreadedMode(bool single_threaded_mode)
  {
    boost::mutex::scoped_lock lock(dmp_status_mutex_);
    single_threaded_mode_ = single_threaded_mode;
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::sendCommand(MessageType& dmp_msg,
                                                                                       const int sequence_number,
                                                                                       bool wait_for_success)
  {
    dmp_status_mutex_.lock();
    if (client_state_ != IDLE)
    {
      dmp_status_mutex_.unlock();
      ROS_WARN("DMP controller client is still active. Waiting before sending next DMP...");
      if(!waitForCompletion())
      {
        return false;
      }
    }

    sequence_number_ = sequence_number;
    ROS_INFO("Sending DMP to controller. Current DMP sequence number is >%i<.", sequence_number_);

    // set sequence number and send DMP
    dmp_msg.dmp.state.seq = sequence_number_;

    ROS_DEBUG("Sending out DMP of type >%s< - Number of subscribers is >%i<", DMPType::getVersionString().c_str(), command_publisher_.getNumSubscribers());
    command_publisher_.publish(dmp_msg);
    ROS_DEBUG("Send out DMP of type >%s<", DMPType::getVersionString().c_str());

    client_state_ = ACTIVE;
    dmp_status_mutex_.unlock();
    if (wait_for_success)
    {
      return waitForCompletion();
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::getControllerStatus(dynamic_movement_primitive::ControllerStatusMsg& dmp_status)
  {
    boost::mutex::scoped_lock lock(dmp_status_mutex_);
    if (sequence_number_ == -1)
    {
      ROS_INFO("Current DMP sequence number is >%i<. Controller has not executed a DMP yet.", sequence_number_);
      return false;
    }
    dmp_status = last_dmp_status_;
    return true;
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::waitForCompletion()
  {

    boost::mutex::scoped_lock lock(dmp_status_mutex_);
    if (single_threaded_mode_)
    {
      lock.unlock();
      while (client_state_ != IDLE && node_handle_.ok())
      {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
      }
    }
    else
    {
      while (client_state_ != IDLE)
      {
        dmp_status_condition_.wait(lock);
      }
    }

    if (last_dmp_status_.status == dynamic_movement_primitive::ControllerStatusMsg::FAILED)
    {
      ROS_ERROR("Received DMP status FAILED. Something went wrong when executing DMP. This should never happen.");
      return false;
    }
    return true;
  }

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_BASE_CLIENT_H_ */
