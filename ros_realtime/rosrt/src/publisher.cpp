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

#include <rosrt/publisher.h>
#include <rosrt/detail/publisher_manager.h>
#include <rosrt/detail/managers.h>
#include <rosrt/init.h>
#include <ros/debug.h>

#include <lockfree/object_pool.h>

#include <boost/thread.hpp>

#ifdef __XENO__
#include <native/task.h>
#endif

namespace rosrt
{
namespace detail
{

bool publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func, CloneFunc clone_func)
{
  return detail::getPublisherManager()->publish(pub, msg, pub_func, clone_func);
}

PublishQueue::PublishQueue(uint32_t size)
: queue_(size)
{
}

bool PublishQueue::push(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func, CloneFunc clone_func)
{
  PubItem i;
  i.pub = pub;
  i.msg = msg;
  i.pub_func = pub_func;
  i.clone_func = clone_func;
  return queue_.push(i);
}

uint32_t PublishQueue::publishAll()
{
  uint32_t count = 0;

  MWSRQueue<PubItem>::Node* it = queue_.popAll();
  while (it)
  {
    // Always clone the message before publishing.  Otherwise, if there's an intraprocess non-realtime subscriber that stores off the messages
    // it could starve the realtime publisher for messages.
    VoidConstPtr clone = it->val.clone_func(it->val.msg);
    it->val.pub_func(it->val.pub, clone);
    it->val.msg.reset();
    it->val.pub = ros::Publisher();
    MWSRQueue<PubItem>::Node* tmp = it;
    it = it->next;

    queue_.free(tmp);

    ++count;
  }

  return count;
}

PublisherManager::PublisherManager(const InitOptions& ops)
: queue_(ops.pubmanager_queue_size)
, pub_count_(0)
, running_(true)
, pub_thread_(boost::bind(&PublisherManager::publishThread, this), "rosrt_publisher", 7) // HACK!!! Assigns to CPU 7
{
}

PublisherManager::~PublisherManager()
{
  cond_mutex_.lock();
  running_ = false;
  cond_.notify_one();
  cond_mutex_.unlock();
  pub_thread_.join();
}

void PublisherManager::publishThread()
{
  while (running_)
  {
    {
      rosrt::mutex::scoped_lock lock(cond_mutex_);
      while (running_ && pub_count_.load() == 0)
      {
        cond_.wait(lock);
      }

      if (!running_)
      {
        return;
      }
    }
#ifdef __XENO__
    // in Xenomai, force a switch to secondary mode here so that
    // publishing doesn't interfere with real-time tasks
    rt_task_set_mode(T_PRIMARY, 0, NULL);
#endif
    uint32_t count = queue_.publishAll();
    pub_count_.fetch_sub(count);
  }
}

bool PublisherManager::publish(const ros::Publisher& pub, const VoidConstPtr& msg, PublishFunc pub_func, CloneFunc clone_func)
{
  if (!queue_.push(pub, msg, pub_func, clone_func))
  {
    return false;
  }

  pub_count_.fetch_add(1);
  cond_.notify_one();

  return true;
}

} // namespace detail
} // namespace rosrt
