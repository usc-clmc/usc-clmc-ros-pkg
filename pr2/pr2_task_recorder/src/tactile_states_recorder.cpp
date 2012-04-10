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

 \file    tactile_states_recorder.cpp

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

// system includes

// ros includes
#include <usc_utilities/bspline.h>
#include <usc_utilities/assert.h>

#include <geometry_msgs/PointStamped.h>

// local includes
#include <task_recorder/tactile_states_recorder.h>

namespace task_recorder
{

const int NUM_TACTILE_SIGNALS = 22;

TactileStatesRecorder::TactileStatesRecorder()
{
}

TactileStatesRecorder::~TactileStatesRecorder()
{
}

bool TactileStatesRecorder::initialize(ros::NodeHandle& node_handle, const std::string& topic_name)
{
    filtered_data_.resize(NUM_TACTILE_SIGNALS);
    unfiltered_data_.resize(NUM_TACTILE_SIGNALS);
    ROS_VERIFY(((filters::MultiChannelFilterBase<double>&)filter_).configure(NUM_TACTILE_SIGNALS, node_handle.getNamespace() + std::string("/HighPass"), node_handle));
    return initializeBase(node_handle, topic_name);
}

bool TactileStatesRecorder::filterAndCrop(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration, int num_samples,
                                           std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& filtered_and_cropped_tactile_states,
                                           std::vector<std::string>& message_names, std::vector<ros::Time>& times, std::vector<double>& data)
{

    int num_tactile_states = recorder_io_.messages_.size();
    if (num_tactile_states == 0)
    {
        ROS_ERROR("Zero tactile states have been logged.");
        return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
    ros::Time our_end_time = recorder_io_.messages_[num_tactile_states - 1].header.stamp;
    int index = 0;
    while (our_end_time.toSec() < 1e-6)
    {
        index++;
        our_end_time = recorder_io_.messages_[num_tactile_states - (1+index)].header.stamp;
    }

    if (our_start_time > start_time || our_end_time < end_time)
    {
        ROS_ERROR("Requested times have not been recorded!");
        ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
        ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
        return false;
    }

    // fit bspline and resample the position and effort trajectories and compute the velocities
    ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples, filtered_and_cropped_tactile_states));
    ROS_ASSERT(static_cast<int>(filtered_and_cropped_tactile_states.size()) == num_samples);

    recorder_io_.messages_.clear();
    recorder_io_.messages_ = filtered_and_cropped_tactile_states;
    return true;
}

bool TactileStatesRecorder::transformMessages(pr2_gripper_sensor_msgs::PR2GripperSensorRawData& tactile_state)
{
    return true;
}

bool TactileStatesRecorder::resample(std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& tactile_states, const ros::Time& start_time, const ros::Time& end_time,
                                      const int num_samples, std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& resampled_tactile_states)
{

    ROS_VERIFY(!tactile_states.empty());

    // first crop
    ROS_VERIFY(crop<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(tactile_states, start_time, end_time));
    // then remove duplicates
    ROS_VERIFY(removeDuplicates<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(tactile_states));

    int num_tactile_states = static_cast<int> (tactile_states.size());

    ROS_VERIFY(accumulator_.initialize(NUM_TACTILE_SIGNALS, num_samples));

    double mean_dt = 0.0;
    std::vector<double> input_vector;
    ROS_VERIFY(computeMeanDtAndInputVector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(tactile_states, mean_dt, input_vector));

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

    double wave_length = interval.toSec() * 2;

    resampled_tactile_states.clear();
    std::string frame_id = tactile_states[0].header.frame_id;
    std::vector<double> input_querry(num_samples);
    for (int i = 0; i < num_samples; i++)
    {
        pr2_gripper_sensor_msgs::PR2GripperSensorRawData gsd;
        gsd.header.frame_id = frame_id;
        gsd.header.seq = i;
        gsd.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
        input_querry[i] = gsd.header.stamp.toSec();
        resampled_tactile_states.push_back(gsd);
    }

    std::vector<double> target_vector;
    std::vector<double> resampled_vector;
    for (int i = 0; i < NUM_TACTILE_SIGNALS; ++i)
    {
        // left finger tip
        target_vector.clear();
        resampled_vector.clear();
        for (int j = 0; j < num_tactile_states; ++j)
        {
            target_vector.push_back(tactile_states[j].left_finger_pad_forces_filtered[i]);
        }
        if (!usc_utilities::resample(input_vector, target_vector, wave_length, input_querry, resampled_vector, false))
        {
            ROS_ERROR("Could not rescale tactile vector %i, splining failed.", i);
            return false;
        }
        for (int j = 0; j < num_samples; ++j)
        {
            resampled_tactile_states[j].left_finger_pad_forces_filtered[i] = resampled_vector[j];
        }

        // left finger tip
        target_vector.clear();
        resampled_vector.clear();
        for (int j = 0; j < num_tactile_states; ++j)
        {
            target_vector.push_back(tactile_states[j].left_finger_pad_forces[i]);
        }
        if (!usc_utilities::resample(input_vector, target_vector, wave_length, input_querry, resampled_vector, false))
        {
            ROS_ERROR("Could not rescale tactile vector %i, splining failed.", i);
            return false;
        }
        for (int j = 0; j < num_samples; ++j)
        {
            resampled_tactile_states[j].left_finger_pad_forces[i] = resampled_vector[j];
        }

        // right finger tip
        target_vector.clear();
        resampled_vector.clear();
        for (int j = 0; j < num_tactile_states; ++j)
        {
            target_vector.push_back(tactile_states[j].right_finger_pad_forces_filtered[i]);
        }
        if (!usc_utilities::resample(input_vector, target_vector, wave_length, input_querry, resampled_vector, false))
        {
            ROS_ERROR("Could not rescale tactile vector %i, splining failed.", i);
            return false;
        }
        for (int j = 0; j < num_samples; ++j)
        {
            resampled_tactile_states[j].right_finger_pad_forces_filtered[i] = resampled_vector[j];
        }

        // right finger tip
        target_vector.clear();
        resampled_vector.clear();
        for (int j = 0; j < num_tactile_states; ++j)
        {
            target_vector.push_back(tactile_states[j].right_finger_pad_forces[i]);
        }
        if (!usc_utilities::resample(input_vector, target_vector, wave_length, input_querry, resampled_vector, false))
        {
            ROS_ERROR("Could not rescale tactile vector %i, splining failed.", i);
            return false;
        }
        for (int j = 0; j < num_samples; ++j)
        {
            resampled_tactile_states[j].right_finger_pad_forces[i] = resampled_vector[j];
        }

        ROS_VERIFY(accumulator_.add(i, resampled_vector));
    }
    return true;
}

void TactileStatesRecorder::getSignalNames(const int signal_index, std::string& signal_name)
{
    signal_name.assign("");
}

void TactileStatesRecorder::setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics)
{
    for (int i=0; i<static_cast<int>(accumulated_trial_statistics.size()); ++i)
    {
        accumulated_trial_statistics[i].name.resize(NUM_TACTILE_SIGNALS);
        accumulated_trial_statistics[i].name[0].assign(std::string("pad_force_back"));
        accumulated_trial_statistics[i].name[1].assign(std::string("pad_force_left_side_back"));
        accumulated_trial_statistics[i].name[2].assign(std::string("pad_force_left_side_front"));
        accumulated_trial_statistics[i].name[3].assign(std::string("pad_force_front_side_left"));
        accumulated_trial_statistics[i].name[4].assign(std::string("pad_force_front_side_right"));
        accumulated_trial_statistics[i].name[5].assign(std::string("pad_force_right_side_front"));
        accumulated_trial_statistics[i].name[6].assign(std::string("pad_force_right_side_back"));
        for(int j=0; j<5; ++j)
        {
            accumulated_trial_statistics[i].name[6+1+(j*3)].assign(std::string("pad_force_left_row_") + usc_utilities::getString(j));
            accumulated_trial_statistics[i].name[6+2+(j*3)].assign(std::string("pad_force_midle_row_") + usc_utilities::getString(j));
            accumulated_trial_statistics[i].name[6+3+(j*3)].assign(std::string("pad_force_right_row_") + usc_utilities::getString(j));
        }
    }
}

bool TactileStatesRecorder::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
{
    vector_of_accumulated_trial_statistics.clear();
    std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics;
    ROS_VERIFY(accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
    setMessageNames(accumulated_trial_statistics);
    vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
    return true;
}

}
