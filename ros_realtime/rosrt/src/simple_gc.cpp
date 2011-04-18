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

#include <rosrt/detail/simple_gc.h>
#include <rosrt/detail/managers.h>
#include <rosrt/init.h>
#include <ros/debug.h>

namespace rosrt
{
namespace detail
{

SimpleGC::SimpleGC(const InitOptions& ops)
: running_(true)
, pool_gc_queue_(ops.gc_queue_size)
, period_(ops.gc_period.toSec())
{
  pool_gc_thread_ = boost::thread(&SimpleGC::gcThread, this);
}

SimpleGC::~SimpleGC()
{
  running_ = false;
  pool_gc_thread_.join();
}

void addPoolToGC(void* pool, SimpleGC::DeleteFunc deleter, SimpleGC::IsDeletableFunc deletable)
{
  getGC()->add(pool, deleter, deletable);
}

void SimpleGC::add(void* pool, DeleteFunc deleter, IsDeletableFunc deletable)
{
  PoolGCItem i;
  i.pool = pool;
  i.deleter = deleter;
  i.is_deletable = deletable;
  pool_gc_queue_.push(i);
}

void SimpleGC::gcThread()
{
  typedef std::vector<PoolGCItem> V_PoolGCItem;
  V_PoolGCItem gc_items;

  while (running_)
  {
    ros::WallDuration(period_).sleep();

    {
      MWSRQueue<PoolGCItem>::Node* it = pool_gc_queue_.popAll();
      while (it)
      {
        gc_items.push_back(it->val);
        MWSRQueue<PoolGCItem>::Node* tmp = it;
        it = it->next;
        pool_gc_queue_.free(tmp);
      }
    }

    {
      for (size_t i = 0; i < gc_items.size();)
      {
        PoolGCItem& item = gc_items[i];
        if (item.is_deletable(item.pool))
        {
          item.deleter(item.pool);
          item = gc_items.back();
          gc_items.pop_back();
        }
        else
        {
          ++i;
        }
      }
    }
  }

  {
    // Once we've stopped running, make sure everything is deleted
    V_PoolGCItem::iterator it = gc_items.begin();
    V_PoolGCItem::iterator end = gc_items.end();
    for (; it != end; ++it)
    {
      PoolGCItem& item = *it;
      if (!item.is_deletable(item.pool))
      {
        ROS_WARN("Pool %p still has allocated blocks.  Deleting anyway.", item.pool);
      }

      item.deleter(item.pool);
    }
  }
}

} // namespace detail
} // namespace rosrt

