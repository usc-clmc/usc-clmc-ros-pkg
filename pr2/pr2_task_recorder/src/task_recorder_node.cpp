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

 \file    task_recorder_node.cpp

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

// system includes

// ros includes
#include <ros/ros.h>

// local includes
#include <task_recorder/joint_states_recorder.h>
#include <task_recorder/gripper_states_recorder.h>
// #include <task_recorder/accelerometer_states_recorder.h>
// #include <task_recorder/tactile_states_recorder.h>
#include <task_recorder/point_cloud_recorder.h>
#include <task_recorder/imu_states_recorder.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_recorder_node");
    ros::NodeHandle node_handle("~");

    task_recorder::JointStatesRecorder joint_state_recorder;
    if (!joint_state_recorder.initialize(node_handle, std::string("/joint_states")))
    {
        ROS_ERROR("Could not initialize joint state recorder.");
        return -1;
    }

//    task_recorder::AccelerometerStatesRecorder r_accelerometer_state_recorder;
//    if (!r_accelerometer_state_recorder.initialize(node_handle, std::string("/r_gripper_sensor_controller/raw_data")))
//    {
//        ROS_ERROR("Could not initialize right gripper accelerometer state recorder.");
//        return -1;
//    }

//    task_recorder::AccelerometerStatesRecorder l_accelerometer_state_recorder;
//    if (!l_accelerometer_state_recorder.initialize(node_handle, std::string("/l_gripper_sensor_controller/raw_data")))
//    {
//        ROS_ERROR("Could not initialize left gripper accelerometer state recorder.");
//        return -1;
//    }

//    task_recorder::TactileStatesRecorder r_tactile_state_recorder;
//    if (!r_tactile_state_recorder.initialize(node_handle, std::string("/r_gripper_sensor_controller/raw_data")))
//    {
//        ROS_ERROR("Could not initialize right gripper tactile state recorder.");
//        return -1;
//    }

//    task_recorder::TactileStatesRecorder l_tactile_state_recorder;
//    if (!l_tactile_state_recorder.initialize(node_handle, std::string("/l_gripper_sensor_controller/raw_data")))
//    {
//        ROS_ERROR("Could not initialize left gripper tactile state recorder.");
//        return -1;
//    }

    task_recorder::GripperStatesRecorder r_gripper_state_recorder;
    if (!r_gripper_state_recorder.initialize(node_handle, std::string("/r_gripper_sensor_controller/raw_data")))
    {
        ROS_ERROR("Could not initialize right gripper state recorder.");
        return -1;
    }

    task_recorder::GripperStatesRecorder l_gripper_state_recorder;
    if (!l_gripper_state_recorder.initialize(node_handle, std::string("/l_gripper_sensor_controller/raw_data")))
    {
        ROS_ERROR("Could not initialize left gripper state recorder.");
        return -1;
    }

    task_recorder::PointCloudRecorder point_cloud_recorder;
    if (!point_cloud_recorder.initialize(node_handle, std::string("/ball_point_cloud")))
    {
        ROS_ERROR("Could not initialize point cloud recorder.");
        return -1;
    }

    task_recorder::ImuStatesRecorder imu_states_recorder;
    if (!imu_states_recorder.initialize(node_handle, std::string("/imu/data")))
    {
        ROS_ERROR("Could not initialize imu states recorder.");
        return -1;
    }

    ros::spin();
    return 1;
}

