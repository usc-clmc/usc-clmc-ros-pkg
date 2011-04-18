/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

  \file    task_manager_test.cpp

  \author  Peter Pastor
  \date    Jun 10, 2010

**********************************************************************/

// system includes

// ros includes
#include <ros/ros.h>
#include <gtest/gtest.h>

// local includes
#include "my_test_task.h"

#include <task_manager/task_manager.h>
#include <task_manager/task.h>


TEST(task_test, INIT_AND_RUN_TASK)
{
    task_manager::TaskManager task_test;
    ros::NodeHandle node_handle;

    // initialize
    EXPECT_TRUE(task_test.initialize());

    // get task by name, bare ptr
    task_manager_interface::Task* task_ptr;
    EXPECT_TRUE(task_test.getTaskByName(std::string("task_manager/TestTask"), task_ptr));
    EXPECT_TRUE(task_ptr->initialize(node_handle));

    // get task by name, boost ptr
    boost::shared_ptr<task_manager_interface::Task> task;
    EXPECT_TRUE(task_test.getTaskByName(std::string("task_manager/TestTask"), task));
    EXPECT_TRUE(task->initialize(node_handle));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
