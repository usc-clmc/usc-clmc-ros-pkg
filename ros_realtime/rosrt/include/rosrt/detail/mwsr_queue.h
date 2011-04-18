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

#ifndef ROSRT_DETAIL_MWSR_QUEUE_H
#define ROSRT_DETAIL_MWSR_QUEUE_H

#include "lockfree/object_pool.h"
#include <ros/atomic.h>

namespace rosrt
{
namespace detail
{

template<typename T>
class MWSRQueue
{
public:
  struct Node
  {
    T val;
    Node* next;
  };

  MWSRQueue(uint32_t size)
  : pool_(size, Node())
  , head_(0)
  {

  }

  bool push(const T& val)
  {
    Node* n = pool_.allocate();
    if (!n)
    {
      return false;
    }

    n->val = val;
    Node* stale_head = head_.load(ros::memory_order_relaxed);
    do
    {
      n->next = stale_head;
    } while(!head_.compare_exchange_weak(stale_head, n, ros::memory_order_release));

    return true;
  }

  Node* popAll()
  {
    Node* last = head_.exchange(0, ros::memory_order_consume);
    Node* first = 0;

    // Reverse the list to get it back in push() order
    while (last)
    {
      Node* tmp = last;
      last = last->next;
      tmp->next = first;
      first = tmp;
    }

    return first;
  }

  void free(Node* node)
  {
    pool_.free(node);
  }

private:
  lockfree::ObjectPool<Node> pool_;
  ros::atomic<Node*> head_;
};

} // namespace detail
} // namespace rosrt

#endif // ROSRT_DETAIL_MWSR_QUEUE_H
