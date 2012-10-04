/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, CLMC Lab, University of Southern California
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

#ifndef ROSRT_THREAD_H_
#define ROSRT_THREAD_H_

#include <boost/utility.hpp>

#ifdef __XENO__
#include <native/task.h>
#else
#include <boost/thread.hpp>
#endif

extern "C"
{
static void thread_proxy(void* arg);

}

namespace rosrt
{

/**
 * Thin wrapper for a "real-time" thread, implementation differs based on platform.
 * Falls back to boost::thread on generic platforms.
 *
 */
class thread: boost::noncopyable
{
private:
#ifdef __XENO__
  RT_TASK thread_;
  boost::function<void ()> thread_fn_;
#else
  boost::thread thread_;
#endif

public:
  explicit thread(boost::function<void ()> thread_fn, const char* name="", const int cpu_id=0)
  {
#ifdef __XENO__
    thread_fn_ = thread_fn;
    int error_code;
    if (error_code = rt_task_spawn(&thread_, name, 0, 0, T_FPU | T_JOINABLE | T_CPU(cpu_id), thread_proxy, &thread_fn_))
    {
      ROS_ERROR("rosrt::thread - Couldn't spawn xenomai thread %s, error code = %d", name, error_code);
    }
    //thread_proxy(&thread_fn_);
#else
    thread_ = boost::thread(thread_fn);
#endif
  }

  ~thread()
  {
  }

  void join()
  {
#ifdef __XENO__
    rt_task_join(&thread_);
#else
    thread_.join();
#endif
  }
};

}

static void thread_proxy(void* arg)
{
  boost::function<void ()>* fn = static_cast<boost::function<void ()>*>(arg);
  (*fn)();
}

#endif /* ROSRT_THREAD_H_ */
