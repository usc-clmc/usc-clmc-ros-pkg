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

#include <gtest/gtest.h>

#include "rosrt/rosrt.h"

#include <ros/ros.h>

#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

#include <boost/thread.hpp>

#ifdef __XENO__
#include <native/task.h>
#include <sys/mman.h>
#endif

using namespace rosrt;

void publishThread(ros::Publisher& pub, bool& done)
{
  uint32_t i = 0;
  std_msgs::UInt32 msg;
  while (!done)
  {
    msg.data = i;
    pub.publish(msg);
    ros::WallDuration(0.0001).sleep();
    ++i;
  }
}

bool filter(const boost::shared_ptr<std_msgs::UInt32 const>& msg, const boost::shared_ptr<std_msgs::UInt64> filtered)
{
  filtered->data = msg->data;

  // some dummy memory allocations
  int* x = new int();
  *x=0;
  delete x;

  return true;
}

TEST(FilteredSubscriber, singleSubscriber)
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::UInt32>("test", 1);
  bool done = false;
  boost::thread t(boost::bind(publishThread, boost::ref(pub), boost::ref(done)));

  FilteredSubscriber<std_msgs::UInt32, std_msgs::UInt64> sub;
  sub.initialize(2, nh, "test", filter);
  //FilteredSubscriber<std_msgs::UInt32, std_msgs::UInt64> sub(2, nh, filter, "test");

  resetThreadAllocInfo();

  uint32_t count = 0;
  int32_t last = -1;
  while (count < 10000)
  {
    std_msgs::UInt64Ptr msg = sub.poll();
    if (msg)
    {
      ASSERT_GT((int32_t)msg->data, last);
      last = msg->data;
      ++count;
      //printf("inc\n");
    }
#ifdef  __XENO__
    rt_task_sleep(1000000);
#endif
  }

  ASSERT_EQ(getThreadAllocInfo().total_ops, 0UL);

  done = true;
  t.join();
}

int main(int argc, char** argv)
{
#ifdef __XENO__
  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_task_shadow(NULL, "test_rt_filtered_subscriber", 0, 0);
#endif

  ros::init(argc, argv, "test_rt_filtered_subscriber");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;
  rosrt::init();

  return RUN_ALL_TESTS();
}
