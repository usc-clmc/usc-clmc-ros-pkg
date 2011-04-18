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
     * @param wait_for_success
     * @return true if command was published successfully, false if not
     */
    bool sendCommand(MessageType& dmp_msg,
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

    /**
     * Halts the currently executing DMP immediately
     * @return
     */
    bool halt(bool wait_for_success = true);

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
    int cur_dmp_id_;

    enum
    {
      IDLE,
      ACTIVE,
      WAITING_FOR_PREEMPT
    } client_state_;

    void statusCallback(const dynamic_movement_primitive::ControllerStatusMsg::ConstPtr& msg);

  };

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::initialize(const std::string& topic_name)
  {
    std::string name = topic_name;
    usc_utilities::appendTrailingSlash(name);

    // advertise dmp commands:
    std::string command_topic_name = name + "command";
    ROS_DEBUG("Creating publisher to topic >%s<.",command_topic_name.c_str());
    command_publisher_ = node_handle_.advertise<MessageType> (command_topic_name, 10);

    // subscribe to dmp status:
    std::string status_topic_name = name + "status";
    ROS_DEBUG("Creating subscriber to topic >%s<.",status_topic_name.c_str());
    status_subcriber_ = node_handle_.subscribe(status_topic_name, 10,
                                               &DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::statusCallback, this);

    client_state_ = IDLE;
    cur_dmp_id_ = 0;

    return true;
  }

template<class DMPType, class MessageType>
  void DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::statusCallback(const dynamic_movement_primitive::ControllerStatusMsg::ConstPtr& msg)
  {
    ROS_DEBUG("Got >%s< controller status callback.", DMPType::getVersionString().c_str());

    if (msg->id != cur_dmp_id_)
    {
      ROS_WARN("Received callback with ID >%i< which is not equal to current DMP id >%i<. Ignoring it.", msg->id, cur_dmp_id_);
      return;
    }

    dmp_status_mutex_.lock();
    last_dmp_status_ = *msg;
    if (msg->status == msg->FINISHED || msg->status == msg->PREEMPTED)
    {
      client_state_ = IDLE;
      dmp_status_condition_.notify_all();
      // cur_dmp_id_ = 0;
    }
    dmp_status_mutex_.unlock();
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
    single_threaded_mode_ = single_threaded_mode;
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::sendCommand(MessageType& dmp_msg,
                                                                                       bool wait_for_success)
  {
    {
      boost::mutex::scoped_lock lock(dmp_status_mutex_);
      if (client_state_ != IDLE)
      {
        ROS_ERROR("DynamicMovementPrimitiveControllerBaseClient: sendCommand() called while previous DMP is still being executed.");
        return false;
      }

      cur_dmp_id_++;
      ROS_INFO("Sending DMP to controller. Current DMP id is >%i<.", cur_dmp_id_);

      // set id and send DMP
      dmp_msg.dmp.state.id = cur_dmp_id_;

      ROS_DEBUG("Sending out DMP of type >%s< - Number of subscribers is >%i<", DMPType::getVersionString().c_str(), command_publisher_.getNumSubscribers());
      command_publisher_.publish(dmp_msg);
      ROS_DEBUG("Send out DMP of type >%s<", DMPType::getVersionString().c_str());
      client_state_ = ACTIVE;
    }
    if (wait_for_success)
    {
      return waitForCompletion();
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DynamicMovementPrimitiveControllerBaseClient<DMPType, MessageType>::halt(bool wait_for_success)
  {

    {
      boost::mutex::scoped_lock lock(dmp_status_mutex_);

      if (cur_dmp_id_ <= 0 || client_state_ == IDLE)
      {
        ROS_WARN("DynamicMovementPrimitiveControllerBaseClient::halt() called when no DMP was in progress");
        return true;
      }

      if (client_state_ == WAITING_FOR_PREEMPT)
      {
        ROS_WARN("DynamicMovementPrimitiveControllerBaseClient::halt() called while already waiting for preemption");
        return false;
      }

      // set id and send DMP
      MessageType dmp_msg;
      dmp_msg.dmp.state.id = cur_dmp_id_;
      command_publisher_.publish(dmp_msg);
      client_state_ = WAITING_FOR_PREEMPT;
    }
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
    if (cur_dmp_id_ == 0)
    {
      ROS_INFO("Current DMP id is >%i<. Controller has not executed a DMP yet.", cur_dmp_id_);
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

    if (last_dmp_status_.status == dynamic_movement_primitive::ControllerStatusMsg::FINISHED
        || last_dmp_status_.status == dynamic_movement_primitive::ControllerStatusMsg::PREEMPTED)
    {
      return true;
    }
    return false;
  }

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_CONTROLLER_BASE_CLIENT_H_ */
