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

 \file    imu_states_recorder.cpp

 \author  Peter Pastor
 \date    Aug 23, 2010

 **********************************************************************/

// system includes
#include <sstream>

// ros includes
#include <usc_utilities/assert.h>

#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PointStamped.h>

// local includes
#include <task_recorder/imu_states_recorder.h>

namespace task_recorder
{

const int NUM_ACC_SIGNALS = 3;

ImuStatesRecorder::ImuStatesRecorder()
{
}

ImuStatesRecorder::~ImuStatesRecorder()
{
}

bool ImuStatesRecorder::initialize(ros::NodeHandle& node_handle, const std::string& topic_name)
{
    filtered_data_.resize(NUM_ACC_SIGNALS);
    unfiltered_data_.resize(NUM_ACC_SIGNALS);
    ROS_VERIFY(((filters::MultiChannelFilterBase<double>&)filter_).configure(NUM_ACC_SIGNALS, node_handle.getNamespace() + std::string("/AccHighPass"), node_handle));
    return initializeBase(node_handle, topic_name);
}

bool ImuStatesRecorder::filterAndCrop(const ros::Time& start_time,
                                      const ros::Time& end_time,
                                      const double movement_duration,
                                      int num_samples,
                                      std::vector<sensor_msgs::Imu>& filtered_and_cropped_imu_states,
                                      std::vector<std::string>& message_names,
                                      std::vector<ros::Time>& times, std::vector<double>& data)
{

    int num_imu_states = recorder_io_.messages_.size();
    if (num_imu_states == 0)
    {
        ROS_ERROR("Zero imu states have been logged.");
        return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
    ros::Time our_end_time = recorder_io_.messages_[num_imu_states - 1].header.stamp;
    int index = 0;
    while (our_end_time.toSec() < 1e-6)
    {
        index++;
        our_end_time = recorder_io_.messages_[num_imu_states - (1+index)].header.stamp;
        ROS_WARN("Invalid time stamps in imu states");
    }

    if (our_start_time > start_time || our_end_time < end_time)
    {
        ROS_ERROR("Requested times have not been recorded!");
        ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
        ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
        return false;
    }

    highPassFilter(recorder_io_.messages_);

    // first crop
    ROS_VERIFY(crop<sensor_msgs::Imu>(recorder_io_.messages_, start_time, end_time));
    // then remove duplicates
    ROS_VERIFY(removeDuplicates<sensor_msgs::Imu>(recorder_io_.messages_));

    // ROS_INFO("Writing raw imu states data...");
    ROS_VERIFY(recorder_io_.writeRawData());
    // ROS_INFO("done");

    // fit bspline and resample the position and effort trajectories and compute the velocities
    ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples, filtered_and_cropped_imu_states));
    ROS_ASSERT(static_cast<int>(filtered_and_cropped_imu_states.size()) == num_samples);

    recorder_io_.messages_.clear();
    recorder_io_.messages_ = filtered_and_cropped_imu_states;
    return true;
}

bool ImuStatesRecorder::transformMessages(sensor_msgs::Imu& imu_state)
{
    return true;
}

void ImuStatesRecorder::highPassFilter(std::vector<sensor_msgs::Imu>& imu_states)
{
    for (unsigned int i=0; i<imu_states.size(); ++i)
    {
        unfiltered_data_[0] = imu_states[i].linear_acceleration.x;
        unfiltered_data_[1] = imu_states[i].linear_acceleration.y;
        unfiltered_data_[2] = imu_states[i].linear_acceleration.z;
        filter_.update(unfiltered_data_, filtered_data_);
        imu_states[i].linear_acceleration.x = filtered_data_[0];
        imu_states[i].linear_acceleration.y = filtered_data_[1];
        imu_states[i].linear_acceleration.z = filtered_data_[2];
    }
}

bool ImuStatesRecorder::resample(std::vector<sensor_msgs::Imu>& imu_states,
                                 const ros::Time& start_time,
                                 const ros::Time& end_time,
                                 const int num_samples,
                                 std::vector<sensor_msgs::Imu>& resampled_imu_states)
{

    ROS_VERIFY(!imu_states.empty());

    int num_imu_states = static_cast<int> (imu_states.size());

    // compute mean dt of the provided time stamps
    double dts[num_imu_states - 1];
    double mean_dt = 0.0;

    for (int i = 0; i < num_imu_states - 1; i++)
    {
        dts[i] = imu_states[i + 1].header.stamp.toSec() - imu_states[i].header.stamp.toSec();
        mean_dt += dts[i];
    }
    mean_dt /= static_cast<double> (num_imu_states - 1);

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

    resampled_imu_states.clear();
    std::string frame_id = imu_states[0].header.frame_id;
    for (int i = 0; i < num_samples; i++)
    {
        sensor_msgs::Imu is;
        is.header.frame_id = frame_id;
        is.header.seq = i;
        is.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());

        bool found = false;
        for (unsigned int j=1; j<recorder_io_.messages_.size(); ++j)
        {
            if((recorder_io_.messages_[j-1].header.stamp < is.header.stamp)
                && (recorder_io_.messages_[j].header.stamp >= is.header.stamp))
            {
                is.orientation = recorder_io_.messages_[j].orientation;
                is.angular_velocity = recorder_io_.messages_[j].angular_velocity;
                is.linear_acceleration = recorder_io_.messages_[j].linear_acceleration;
                found = true;
                break;
            }
        }

        // TODO: double check the neirest neighbor approach again...
        // it seems that the last message is not included...
        if(!found)
        {
            is.orientation = resampled_imu_states.back().orientation;
            is.angular_velocity = resampled_imu_states.back().angular_velocity;
            is.linear_acceleration = resampled_imu_states.back().linear_acceleration;
            found = true;
        }
        ROS_ASSERT(found);

        resampled_imu_states.push_back(is);
    }


    return true;
}

void ImuStatesRecorder::getSignalNames(const int signal_index, std::string& signal_name)
{
    signal_name.assign("");
}

void ImuStatesRecorder::setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics)
{
}

bool ImuStatesRecorder::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
{
    return true;
}

}
