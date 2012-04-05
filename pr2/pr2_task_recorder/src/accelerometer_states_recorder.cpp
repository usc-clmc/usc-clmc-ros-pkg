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

 \file    accelerometer_states_recorder.cpp

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

// system includes
#include <sstream>

// ros includes
#include <usc_utilities/bspline.h>
#include <usc_utilities/assert.h>

#include <geometry_msgs/PointStamped.h>


// local includes
#include <task_recorder/accelerometer_states_recorder.h>

namespace task_recorder
{

const int NUM_ACCELEROMETER_SIGNALS = 3;

AccelerometerStatesRecorder::AccelerometerStatesRecorder()
{
}

AccelerometerStatesRecorder::~AccelerometerStatesRecorder()
{
}

bool AccelerometerStatesRecorder::initialize(ros::NodeHandle& node_handle, const std::string& topic_name)
{
    return initializeBase(node_handle, topic_name);
}

bool AccelerometerStatesRecorder::filterAndCrop(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration, int num_samples,
                                        std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& filtered_and_cropped_accelerometer_states,
                                        std::vector<std::string>& message_names, std::vector<ros::Time>& times, std::vector<double>& data)
{

    bool right_arm = true;
    ROS_VERIFY(!message_names.empty());
    if(message_names[0].compare("left"))
    {
        right_arm = false;
    }

    int num_accelerometer_states = recorder_io_.messages_.size();
    if (num_accelerometer_states == 0)
    {
        ROS_ERROR("Zero accelerometer states have been logged.");
        return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
    ros::Time our_end_time = recorder_io_.messages_[num_accelerometer_states - 1].header.stamp;
    int index = 0;
    while (our_end_time.toSec() < 1e-6)
    {
        index++;
        our_end_time = recorder_io_.messages_[num_accelerometer_states - (1+index)].header.stamp;
    }

    if (our_start_time > start_time || our_end_time < end_time)
    {
        ROS_ERROR("Requested times have not been recorded!");
        ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
        ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
        return false;
    }

    // fit bspline and resample the position and effort trajectories and compute the velocities
    ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples, filtered_and_cropped_accelerometer_states));
    ROS_ASSERT(static_cast<int>(filtered_and_cropped_accelerometer_states.size()) == num_samples);

    recorder_io_.messages_.clear();
    recorder_io_.messages_ = filtered_and_cropped_accelerometer_states;
    return true;
}

bool AccelerometerStatesRecorder::findAndSetPeakTimes(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration,
                                                      ros::Time& first_peak_time, ros::Time& second_peak_time)
{
    first_peak_time = ros::Time(0);
    second_peak_time = ros::Time(0);
    double first_max = 0;
    double second_max = 0;
    bool first_is_set = false;
    bool second_is_set = false;
    for (int i=0; i<static_cast<int>(recorder_io_.messages_.size()); ++i)
    {
        double sum = fabs(recorder_io_.messages_[i].acc_x_filtered)
                        + fabs(recorder_io_.messages_[i].acc_y_filtered)
                        + fabs(recorder_io_.messages_[i].acc_z_filtered);
        if((recorder_io_.messages_[i].header.stamp >= start_time)
                && (recorder_io_.messages_[i].header.stamp < start_time + ros::Duration(movement_duration)))
        {
            if(sum > first_max)
            {
                first_max = sum;
                first_peak_time = recorder_io_.messages_[i].header.stamp;
//                if(!first_is_set)
//                {
//                    ROS_INFO("First spike is set.");
//                }
                first_is_set = true;
            }
        }
        if((recorder_io_.messages_[i].header.stamp >= start_time + ros::Duration(movement_duration))
                && (recorder_io_.messages_[i].header.stamp < end_time))
        {
            if(sum > second_max)
            {
                second_max = sum;
                second_peak_time = recorder_io_.messages_[i].header.stamp;
//                if(!second_is_set)
//                {
                //                    ROS_INFO("Second spike is set.");
//                }
                second_is_set = true;
            }
        }
    }
    if(!first_is_set)
    {
        ROS_INFO("First spike NOT set.");
    }
    if(!second_is_set)
    {
        ROS_INFO("Second spike NOT set.");
    }

//    return (first_is_set && second_is_set);
    return true;
}

void AccelerometerStatesRecorder::msgCallback(boost::shared_ptr<geometry_msgs::Vector3Stamped>& vector)
{
  geometry_msgs::Vector3Stamped vector_out;
  try
  {
    listener_.transformVector(std::string("/torso_lift_link"), *vector, vector_out);
  }
  catch (tf::TransformException &ex)
  {
    // ROS_ERROR("%s", ex.what());
  }
  *vector = vector_out;
};


bool AccelerometerStatesRecorder::transformMessages(pr2_gripper_sensor_msgs::PR2GripperSensorRawData& accelerometer_state)
{

    geometry_msgs::Vector3Stamped vector_stamped_in;
    geometry_msgs::Vector3Stamped vector_stamped_out;
    vector_stamped_in.header = accelerometer_state.header;
    vector_stamped_in.vector.x = accelerometer_state.acc_x_filtered;
    vector_stamped_in.vector.y = accelerometer_state.acc_y_filtered;
    vector_stamped_in.vector.z = accelerometer_state.acc_z_filtered;

    tf::StampedTransform transform;
    try
    {
        listener_.waitForTransform(vector_stamped_in.header.frame_id, std::string("/torso_lift_link"), vector_stamped_in.header.stamp, ros::Duration(0.05), ros::Duration(0.005));
        listener_.transformVector(std::string("/torso_lift_link"), vector_stamped_in, vector_stamped_out);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
    }

    accelerometer_state.header = vector_stamped_out.header;
    accelerometer_state.acc_x_filtered = vector_stamped_out.vector.x;
    accelerometer_state.acc_y_filtered = vector_stamped_out.vector.y;
    accelerometer_state.acc_z_filtered = vector_stamped_out.vector.z;

    return true;
}

bool AccelerometerStatesRecorder::resample(std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& accelerometer_states,
                                           const ros::Time& start_time, const ros::Time& end_time, const int num_samples,
                                           std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& resampled_accelerometer_states)
{

    ROS_VERIFY(!accelerometer_states.empty());

    // first crop
    ROS_VERIFY(crop<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(accelerometer_states, start_time, end_time));
    // then remove duplicates
    ROS_VERIFY(removeDuplicates<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(accelerometer_states));

    int num_accelerometer_states = static_cast<int> (accelerometer_states.size());

    ROS_VERIFY(accumulator_.initialize(NUM_ACCELEROMETER_SIGNALS, num_samples));

    // compute mean dt of the provided time stamps
    double dts[num_accelerometer_states - 1];
    double mean_dt = 0.0;

    std::vector<double> input_vector(num_accelerometer_states);
    input_vector[0] = accelerometer_states[0].header.stamp.toSec();
    for (int i = 0; i < num_accelerometer_states - 1; i++)
    {
        dts[i] = accelerometer_states[i + 1].header.stamp.toSec() - accelerometer_states[i].header.stamp.toSec();
        mean_dt += dts[i];
        input_vector[i + 1] = input_vector[i] + dts[i];
    }
    mean_dt /= static_cast<double> (num_accelerometer_states - 1);

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

    double wave_length = interval.toSec() * static_cast<double>(2.0);

    resampled_accelerometer_states.clear();
    std::string frame_id = accelerometer_states[0].header.frame_id;
    std::vector<double> input_querry(num_samples);
    for (int i = 0; i < num_samples; i++)
    {
        pr2_gripper_sensor_msgs::PR2GripperSensorRawData as;
        as.header.frame_id = frame_id;
        as.header.seq = i;
        as.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
        input_querry[i] = as.header.stamp.toSec();
        resampled_accelerometer_states.push_back(as);
    }

    std::vector<double> acceleration_target_vector_x;
    std::vector<double> acceleration_target_vector_y;
    std::vector<double> acceleration_target_vector_z;
    std::vector<double> acceleration_vector_resampled_x;
    std::vector<double> acceleration_vector_resampled_y;
    std::vector<double> acceleration_vector_resampled_z;
    for (int j = 0; j < num_accelerometer_states; ++j)
    {
        // ROS_INFO("x = %f", accelerometer_states[j].samples[0].x);
        acceleration_target_vector_x.push_back(accelerometer_states[j].acc_x_filtered);
        acceleration_target_vector_y.push_back(accelerometer_states[j].acc_y_filtered);
        acceleration_target_vector_z.push_back(accelerometer_states[j].acc_z_filtered);
    }

    if(!usc_utilities::resample(input_vector, acceleration_target_vector_x, wave_length, input_querry, acceleration_vector_resampled_x, false))
    {
        ROS_ERROR("Could not rescale acceleration vector x, splining failed.");
        return false;
    }
    if(!usc_utilities::resample(input_vector, acceleration_target_vector_y, wave_length, input_querry, acceleration_vector_resampled_y, false))
    {
        ROS_ERROR("Could not rescale acceleration vector y, splining failed.");
        return false;
    }
    if(!usc_utilities::resample(input_vector, acceleration_target_vector_z, wave_length, input_querry, acceleration_vector_resampled_z, false))
    {
        ROS_ERROR("Could not rescale acceleration vector z, splining failed.");
        return false;
    }

    for (int j = 0; j < num_samples; ++j)
    {
        resampled_accelerometer_states[j].acc_x_filtered = acceleration_vector_resampled_x[j];
        resampled_accelerometer_states[j].acc_y_filtered = acceleration_vector_resampled_y[j];
        resampled_accelerometer_states[j].acc_z_filtered = acceleration_vector_resampled_z[j];
    }

    ROS_VERIFY(accumulator_.add(0, acceleration_vector_resampled_x));
    ROS_VERIFY(accumulator_.add(1, acceleration_vector_resampled_y));
    ROS_VERIFY(accumulator_.add(2, acceleration_vector_resampled_z));

    return true;
}

void AccelerometerStatesRecorder::getSignalNames(const int signal_index, std::string& signal_name)
{
    signal_name.assign("");
}

void AccelerometerStatesRecorder::setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics)
{
    for (int i=0; i<static_cast<int>(accumulated_trial_statistics.size()); ++i)
    {
        accumulated_trial_statistics[i].name.resize(NUM_ACCELEROMETER_SIGNALS);
        accumulated_trial_statistics[i].name[0].assign(std::string("gripper_accelerometer_x"));
        accumulated_trial_statistics[i].name[1].assign(std::string("gripper_accelerometer_y"));
        accumulated_trial_statistics[i].name[2].assign(std::string("gripper_accelerometer_z"));
    }
}

bool AccelerometerStatesRecorder::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
{
    vector_of_accumulated_trial_statistics.clear();
    std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics;
    ROS_VERIFY(accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
    setMessageNames(accumulated_trial_statistics);
    vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
    return true;
}


}
