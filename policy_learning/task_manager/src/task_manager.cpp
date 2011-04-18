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

  \file    task_manager.cpp

  \author  Peter Pastor
  \date    Jun 10, 2010

**********************************************************************/

// system includes
#include <string>

// ros includes
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// local includes
#include <task_manager/task_manager.h>
#include <task_manager/task.h>

namespace task_manager {

TaskManager::TaskManager() : initialized_(false)
{
}

TaskManager::~TaskManager()
{

}

bool TaskManager::initialize()
{
    task_loader_.reset(new pluginlib::ClassLoader<task_manager_interface::Task>("task_manager", "task_manager_interface::Task"));

    return (initialized_ = true);
}

bool TaskManager::getTaskByName(const std::string task_name, task_manager_interface::Task*& task)
{
    ROS_ASSERT(initialized_);

    try
    {
        if (task_loader_->isClassAvailable(task_name))
        {
            task = task_loader_->createClassInstance(task_name);
        }
        else
        {
            ROS_ERROR("No class available in package/class %s.", task_name.c_str());
            return false;
        }
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      return false;
    }

    return true;
}

bool TaskManager::getTaskByName(const std::string task_name, boost::shared_ptr<task_manager_interface::Task>& task)
{
    ROS_ASSERT(initialized_);

    task_manager_interface::Task* task_ptr;
    if (!getTaskByName(task_name, task_ptr))
    {
        return false;
    }
    task.reset(task_ptr);
    return true;
}


}

