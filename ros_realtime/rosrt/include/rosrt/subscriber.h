/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef ROSRT_SUBSCRIBER_H
#define ROSRT_SUBSCRIBER_H

#include <lockfree/object_pool.h>
#include "detail/pool_gc.h"

#include <ros/atomic.h>
#include <ros/ros.h>

namespace ros
{
class CallbackQueueInterface;
} // namespace ros

namespace rosrt
{

namespace detail
{
  ros::CallbackQueueInterface* getSubscriberCallbackQueue();
} // namespace detail

/**
 * \brief A lock-free subscriber.  Allows you to receive ROS messages inside a realtime thread.
 *
 * This subscriber works in a polling manner rather than the usual callback-based mechanism, e.g.:
\verbatim
Subscriber<Msg> sub(2, nh, "my_topic");
while (true)
{
  MsgConstPtr msg = sub.poll();
  if (msg)
  {
    // do something with msg
    ...
  }
}
\endverbatim
 */
template<typename M>
class Subscriber
{
public:
  /**
   * \brief Default constructor.  You must call initialize() before doing anything else if you use this constructor.
   */
  Subscriber()
  : pool_(0)
  {
  }

  /**
   * \brief Constructor with initialization.  Call subscribe() to subscribe to a topic.
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   */
  Subscriber(uint32_t message_pool_size)
  : pool_(0)
  {
    initialize(message_pool_size);
  }

  /**
   * \brief Constructor with initialization and subscription
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   * \param nh The ros::NodeHandle to use to subscribe
   * \param topic The topic to subscribe on
   * \param [optional] transport_hints the transport hints to use
   */
  Subscriber(uint32_t message_pool_size, ros::NodeHandle& nh, const std::string& topic, const ros::TransportHints& transport_hints = ros::TransportHints())
  : pool_(0)
  {
    initialize(message_pool_size);
    subscribe(nh, topic, transport_hints);
  }

  ~Subscriber()
  {
    M const* latest = latest_.exchange(0);
    if (latest)
    {
      pool_->free(latest);
    }

    detail::addPoolToGC((void*)pool_, detail::deletePool<M>, detail::poolIsDeletable<M>);
  }

  /**
   * \brief Initialize this subscribe.  Only use with the default constructor.
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   */
  void initialize(uint32_t message_pool_size)
  {
    ROS_ASSERT(message_pool_size > 1);
    ROS_ASSERT(!pool_);
    pool_ = new lockfree::ObjectPool<M>();
    pool_->initialize(message_pool_size, M());
    latest_.store(0);
  }

  /**
   * \brief Initialize this subscribe.  Only use with the default constructor.
   * \param message_pool_size The size of the message pool to use.  If this pool fills up no more messages
   * will be received until some messages are freed.
   * \param nh The ros::NodeHandle to use to subscribe
   * \param topic The topic to subscribe on
   * \param [optional] transport_hints the transport hints to use
   * \return Whether or not we successfully subscribed
   */
  bool initialize(uint32_t message_pool_size, ros::NodeHandle& nh, const std::string& topic, const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    initialize(message_pool_size);
    return subscribe(nh, topic, transport_hints);
  }

  /**
   * \brief Subscribe to a topic
   * \param nh The ros::NodeHandle to use to subscribe
   * \param topic The topic to subscribe on
   * \param [optional] transport_hints the transport hints to use
   * \return Whether or not we successfully subscribed
   */
  bool subscribe(ros::NodeHandle& nh, const std::string& topic, const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
#ifdef ROS_NEW_SERIALIZATION_API
    ops.template init<M>(topic, 1, boost::bind(&Subscriber::callback, this, _1), boost::bind(&lockfree::ObjectPool<M>::allocateShared, pool_));
#else
    ops.template init<M>(topic, 1, boost::bind(&Subscriber::callback, this, _1));
#endif
    ops.callback_queue = detail::getSubscriberCallbackQueue();
    sub_ = nh.subscribe(ops);
    return (bool)sub_;
  }

  /**
   * \brief Retrieve the newest message received.
   *
   * The same message will only be returned once, i.e:
\verbatim
<msg received>
msg = poll(); // Returns a valid message
msg = poll(); // Returns NULL
\endverbatim
   */
  boost::shared_ptr<M const> poll()
  {
    M const* latest = latest_.exchange(0);
    if (!latest)
    {
      return boost::shared_ptr<M const>();
    }

    boost::shared_ptr<M const> ptr = pool_->makeShared(latest);
    if (!ptr)
    {
      pool_->free(latest);
      return boost::shared_ptr<M const>();
    }

    return ptr;
  }

private:
  void callback(const boost::shared_ptr<M const>& msg)
  {
    M const* latest = 0;
    // If our pool doesn't own this message (due to multiple subscribers on the same topic)
    // make a copy
    if (!pool_->owns(msg.get()))
    {
      M* copy = pool_->allocate();

      if (!copy)
      {
        return;
      }

      *copy = *msg;
      latest = copy;
    }
    else
    {
      latest = pool_->removeShared(msg);
    }

    M const* old = latest_.exchange(latest);
    if (old)
    {
      pool_->free(old);
    }
  }

  ros::atomic<M const*> latest_;

  lockfree::ObjectPool<M>* pool_;
  ros::Subscriber sub_;
};

} // namespace rosrt

#endif // ROSRT_SUBSCRIBER_H
