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

 \file    gripper_states_recorder.cpp

 \author  Peter Pastor
 \date    Aug 27, 2010

 **********************************************************************/

// system includes
#include <sstream>

// ros includes
#include <usc_utilities/bspline.h>
#include <usc_utilities/assert.h>

#include <geometry_msgs/PointStamped.h>

// local includes
#include <task_recorder/gripper_states_recorder.h>

namespace task_recorder
{

const int NUM_ACCELEROMETER_SIGNALS = 3;
const int NUM_TACTILE_SIGNALS = 22;

GripperStatesRecorder::GripperStatesRecorder()
{
}

GripperStatesRecorder::~GripperStatesRecorder()
{
}

bool GripperStatesRecorder::initialize(ros::NodeHandle& node_handle, const std::string& topic_name)
{
    return initializeBase(node_handle, topic_name);
}

bool GripperStatesRecorder::filterAndCrop(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration, int num_samples,
                                          std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& filtered_and_cropped_gripper_states,
                                          std::vector<std::string>& message_names, std::vector<ros::Time>& times, std::vector<double>& data)
{

    int num_gripper_states = recorder_io_.messages_.size();
    if (num_gripper_states == 0)
    {
        ROS_ERROR("Zero gripper states have been logged.");
        return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
    ros::Time our_end_time = recorder_io_.messages_[num_gripper_states - 1].header.stamp;
    int index = 0;
    while (our_end_time.toSec() < 1e-6)
    {
        index++;
        our_end_time = recorder_io_.messages_[num_gripper_states - (1+index)].header.stamp;
        ROS_WARN("Invalid time stamps in gripper states");
    }

    if (our_start_time > start_time || our_end_time < end_time)
    {
        ROS_ERROR("Requested times have not been recorded!");
        ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
        ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
        return false;
    }

    // first crop
    ROS_VERIFY(crop<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(recorder_io_.messages_, start_time, end_time));
    // then remove duplicates
    ROS_VERIFY(removeDuplicates<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(recorder_io_.messages_));

    // ROS_INFO("Writing raw gripper data...");
    ROS_VERIFY(recorder_io_.writeRawData());
    // ROS_INFO("done");

    // fit bspline and resample the position and effort trajectories and compute the velocities
    std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData> accelerometer_states;
    ROS_VERIFY(resampleAccelerometerMessages(recorder_io_.messages_, start_time, end_time, num_samples, accelerometer_states));
    ROS_ASSERT(static_cast<int>(accelerometer_states.size()) == num_samples);

    std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData> pressure_states;
    ROS_VERIFY(resamplePressureMessages(recorder_io_.messages_, start_time, end_time, num_samples, pressure_states));
    ROS_ASSERT(static_cast<int>(pressure_states.size()) == num_samples);

    ROS_VERIFY(fillGripperState(accelerometer_states, pressure_states, filtered_and_cropped_gripper_states));

    recorder_io_.messages_.clear();
    recorder_io_.messages_ = filtered_and_cropped_gripper_states;

    return true;
}

//bool GripperStatesRecorder::findAndSetPeakTimes(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration,
//                                                      ros::Time& first_peak_time, ros::Time& second_peak_time)
//{
//    first_peak_time = ros::Time(0);
//    second_peak_time = ros::Time(0);
//    double first_max = 0;
//    double second_max = 0;
//    bool first_is_set = false;
//    bool second_is_set = false;
//    for (int i=0; i<static_cast<int>(recorder_io_.messages_.size()); ++i)
//    {
//        double sum = fabs(recorder_io_.messages_[i].acc_x_filtered)
//                        + fabs(recorder_io_.messages_[i].acc_y_filtered)
//                        + fabs(recorder_io_.messages_[i].acc_z_filtered);
//        if((recorder_io_.messages_[i].header.stamp >= start_time)
//                && (recorder_io_.messages_[i].header.stamp < start_time + ros::Duration(movement_duration)))
//        {
//            if(sum > first_max)
//            {
//                first_max = sum;
//                first_peak_time = recorder_io_.messages_[i].header.stamp;
////                if(!first_is_set)
////                {
////                    ROS_INFO("First spike is set.");
////                }
//                first_is_set = true;
//            }
//        }
//        if((recorder_io_.messages_[i].header.stamp >= start_time + ros::Duration(movement_duration))
//                && (recorder_io_.messages_[i].header.stamp < end_time))
//        {
//            if(sum > second_max)
//            {
//                second_max = sum;
//                second_peak_time = recorder_io_.messages_[i].header.stamp;
////                if(!second_is_set)
////                {
//                //                    ROS_INFO("Second spike is set.");
////                }
//                second_is_set = true;
//            }
//        }
//    }
//    if(!first_is_set)
//    {
//        ROS_INFO("First spike NOT set.");
//    }
//    if(!second_is_set)
//    {
//        ROS_INFO("Second spike NOT set.");
//    }
//
////    return (first_is_set && second_is_set);
//    return true;
//}

void GripperStatesRecorder::msgCallback(boost::shared_ptr<geometry_msgs::Vector3Stamped>& vector)
{
  geometry_msgs::Vector3Stamped vector_out;
  try
  {
    listener_.transformVector(std::string("/torso_lift_link"), *vector, vector_out);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  *vector = vector_out;
};


bool GripperStatesRecorder::transformMessages(pr2_gripper_sensor_msgs::PR2GripperSensorRawData& gripper_state)
{

    geometry_msgs::Vector3Stamped vector_stamped_in;
    geometry_msgs::Vector3Stamped vector_stamped_out;
    vector_stamped_in.header = gripper_state.header;
    vector_stamped_in.vector.x = gripper_state.acc_x_filtered;
    vector_stamped_in.vector.y = gripper_state.acc_y_filtered;
    vector_stamped_in.vector.z = gripper_state.acc_z_filtered;

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

    gripper_state.header = vector_stamped_out.header;
    gripper_state.acc_x_filtered = vector_stamped_out.vector.x;
    gripper_state.acc_y_filtered = vector_stamped_out.vector.y;
    gripper_state.acc_z_filtered = vector_stamped_out.vector.z;

    return true;
}

bool GripperStatesRecorder::resampleAccelerometerMessages(std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& gripper_states,
                                                          const ros::Time& start_time, const ros::Time& end_time, const int num_samples,
                                                          std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& resampled_gripper_states)
{

    ROS_VERIFY(!gripper_states.empty());

    int num_gripper_states = static_cast<int> (gripper_states.size());

    // compute mean dt of the provided time stamps
    double dts[num_gripper_states - 1];
    double mean_dt = 0.0;

    std::vector<double> input_vector(num_gripper_states);
    input_vector[0] = gripper_states[0].header.stamp.toSec();
    for (int i = 0; i < num_gripper_states - 1; i++)
    {
        dts[i] = gripper_states[i + 1].header.stamp.toSec() - gripper_states[i].header.stamp.toSec();
        mean_dt += dts[i];
        input_vector[i + 1] = input_vector[i] + dts[i];
    }
    mean_dt /= static_cast<double> (num_gripper_states - 1);

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

    double wave_length = interval.toSec() * static_cast<double>(2.0);

    resampled_gripper_states.clear();
    std::string frame_id = gripper_states[0].header.frame_id;
    std::vector<double> input_querry(num_samples);
    for (int i = 0; i < num_samples; i++)
    {
        pr2_gripper_sensor_msgs::PR2GripperSensorRawData as;
        as.header.frame_id = frame_id;
        as.header.seq = i;
        as.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
        input_querry[i] = as.header.stamp.toSec();
        resampled_gripper_states.push_back(as);
    }

    std::vector<double> acceleration_target_vector_x;
    std::vector<double> acceleration_target_vector_y;
    std::vector<double> acceleration_target_vector_z;
    std::vector<double> acceleration_vector_resampled_x;
    std::vector<double> acceleration_vector_resampled_y;
    std::vector<double> acceleration_vector_resampled_z;
    for (int j = 0; j < num_gripper_states; ++j)
    {
        // ROS_INFO("x = %f", gripper_states[j].samples[0].x);
        acceleration_target_vector_x.push_back(gripper_states[j].acc_x_filtered);
        acceleration_target_vector_y.push_back(gripper_states[j].acc_y_filtered);
        acceleration_target_vector_z.push_back(gripper_states[j].acc_z_filtered);
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
        resampled_gripper_states[j].acc_x_filtered = acceleration_vector_resampled_x[j];
        resampled_gripper_states[j].acc_y_filtered = acceleration_vector_resampled_y[j];
        resampled_gripper_states[j].acc_z_filtered = acceleration_vector_resampled_z[j];
    }


    return true;
}

bool GripperStatesRecorder::downSample(const std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& tactile_states,
                                       std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& down_sampled_tactile_states)
{

    int counter = 0;
    for(int i=0; i<static_cast<int>(tactile_states.size()); i++)
    {
        if((counter % 20) == 0)
        {
            down_sampled_tactile_states.push_back(tactile_states[i]);
        }
    }

    return true;
}

bool GripperStatesRecorder::resamplePressureMessages(std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& gripper_states, const ros::Time& start_time, const ros::Time& end_time,
                                                     const int num_samples, std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& resampled_tactile_states)
{

    ROS_VERIFY(!gripper_states.empty());

    std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData> down_sampled_tactile_states;
    ROS_VERIFY(downSample(gripper_states, down_sampled_tactile_states));

    int num_tactile_states = static_cast<int> (down_sampled_tactile_states.size());

    double mean_dt = 0.0;
    std::vector<double> input_vector;
    ROS_VERIFY(computeMeanDtAndInputVector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(down_sampled_tactile_states, mean_dt, input_vector));

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

    double wave_length = interval.toSec() * 2;

    resampled_tactile_states.clear();
    std::string frame_id = down_sampled_tactile_states[0].header.frame_id;
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
            target_vector.push_back(down_sampled_tactile_states[j].left_finger_pad_forces_filtered[i]);
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
            target_vector.push_back(down_sampled_tactile_states[j].left_finger_pad_forces[i]);
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
            target_vector.push_back(down_sampled_tactile_states[j].right_finger_pad_forces_filtered[i]);
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
            target_vector.push_back(down_sampled_tactile_states[j].right_finger_pad_forces[i]);
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

    }
    return true;
}

bool GripperStatesRecorder::fillGripperState(const std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& accelerometer_states,
                                             const std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& pressure_states,
                                             std::vector<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>& filtered_gripper_states)
{

    ROS_ASSERT(accelerometer_states.size() == pressure_states.size());
    ROS_VERIFY(accumulator_.initialize(NUM_ACCELEROMETER_SIGNALS+NUM_TACTILE_SIGNALS+NUM_TACTILE_SIGNALS, static_cast<int>(accelerometer_states.size())));

    std::vector<double> acceleration_vector_resampled_x;
    std::vector<double> acceleration_vector_resampled_y;
    std::vector<double> acceleration_vector_resampled_z;

    for (int i=0; i<static_cast<int>(accelerometer_states.size()); ++i)
    {
        pr2_gripper_sensor_msgs::PR2GripperSensorRawData gs;
        gs.header = accelerometer_states[i].header;
        gs.acc_x_filtered = accelerometer_states[i].acc_x_filtered;
        acceleration_vector_resampled_x.push_back(accelerometer_states[i].acc_x_filtered);
        gs.acc_y_filtered = accelerometer_states[i].acc_y_filtered;
        acceleration_vector_resampled_y.push_back(accelerometer_states[i].acc_y_filtered);
        gs.acc_z_filtered = accelerometer_states[i].acc_z_filtered;
        acceleration_vector_resampled_z.push_back(accelerometer_states[i].acc_z_filtered);
        gs.left_finger_pad_forces = pressure_states[i].left_finger_pad_forces;
        gs.right_finger_pad_forces = pressure_states[i].right_finger_pad_forces;
        filtered_gripper_states.push_back(gs);
    }

    ROS_VERIFY(accumulator_.add(0, acceleration_vector_resampled_x));
    ROS_VERIFY(accumulator_.add(1, acceleration_vector_resampled_y));
    ROS_VERIFY(accumulator_.add(2, acceleration_vector_resampled_z));

    std::vector<double> left_tactile_data_vector;
    std::vector<double> right_tactile_data_vector;
    for (int i = 0; i < NUM_TACTILE_SIGNALS; ++i)
    {
        left_tactile_data_vector.clear();
        right_tactile_data_vector.clear();
        for (int j=0; j<static_cast<int>(filtered_gripper_states.size()); ++j)
        {
            left_tactile_data_vector.push_back(filtered_gripper_states[j].left_finger_pad_forces[i]);
            right_tactile_data_vector.push_back(filtered_gripper_states[j].right_finger_pad_forces[i]);
        }
        ROS_VERIFY(accumulator_.add(3+i, right_tactile_data_vector));
        ROS_VERIFY(accumulator_.add(3+NUM_TACTILE_SIGNALS+i, left_tactile_data_vector));
    }

    return true;
}

void GripperStatesRecorder::getSignalNames(const int signal_index, std::string& signal_name)
{
    signal_name.assign("");
}

void GripperStatesRecorder::setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics)
{
    for (int i=0; i<static_cast<int>(accumulated_trial_statistics.size()); ++i)
    {
        accumulated_trial_statistics[i].name.resize(NUM_ACCELEROMETER_SIGNALS + NUM_TACTILE_SIGNALS + NUM_TACTILE_SIGNALS);

        accumulated_trial_statistics[i].name[0].assign(std::string("gripper_acc_x"));
        accumulated_trial_statistics[i].name[1].assign(std::string("gripper_acc_y"));
        accumulated_trial_statistics[i].name[2].assign(std::string("gripper_acc_z"));

        accumulated_trial_statistics[i].name[3].assign(std::string("r_pad_force_back"));
        accumulated_trial_statistics[i].name[4].assign(std::string("r_pad_force_left_side_back"));
        accumulated_trial_statistics[i].name[5].assign(std::string("r_pad_force_left_side_front"));
        accumulated_trial_statistics[i].name[6].assign(std::string("r_pad_force_front_side_left"));
        accumulated_trial_statistics[i].name[7].assign(std::string("r_pad_force_front_side_right"));
        accumulated_trial_statistics[i].name[8].assign(std::string("r_pad_force_right_side_front"));
        accumulated_trial_statistics[i].name[9].assign(std::string("r_pad_force_right_side_back"));

        for(int j=0; j<5; ++j)
        {
            accumulated_trial_statistics[i].name[10+0+(j*3)].assign(std::string("r_pad_force_left_row_") + usc_utilities::getString(j));
            accumulated_trial_statistics[i].name[10+1+(j*3)].assign(std::string("r_pad_force_midle_row_") + usc_utilities::getString(j));
            accumulated_trial_statistics[i].name[10+2+(j*3)].assign(std::string("r_pad_force_right_row_") + usc_utilities::getString(j));
        }

        accumulated_trial_statistics[i].name[3 + NUM_TACTILE_SIGNALS].assign(std::string("l_pad_force_back"));
        accumulated_trial_statistics[i].name[4 + NUM_TACTILE_SIGNALS].assign(std::string("l_pad_force_left_side_back"));
        accumulated_trial_statistics[i].name[5 + NUM_TACTILE_SIGNALS].assign(std::string("l_pad_force_left_side_front"));
        accumulated_trial_statistics[i].name[6 + NUM_TACTILE_SIGNALS].assign(std::string("l_pad_force_front_side_left"));
        accumulated_trial_statistics[i].name[7 + NUM_TACTILE_SIGNALS].assign(std::string("l_pad_force_front_side_right"));
        accumulated_trial_statistics[i].name[8 + NUM_TACTILE_SIGNALS].assign(std::string("l_pad_force_right_side_front"));
        accumulated_trial_statistics[i].name[9 + NUM_TACTILE_SIGNALS].assign(std::string("l_pad_force_right_side_back"));

        for(int j=0; j<5; ++j)
        {
            accumulated_trial_statistics[i].name[10+NUM_TACTILE_SIGNALS+0+(j*3)].assign(std::string("l_pad_force_left_row_") + usc_utilities::getString(j));
            accumulated_trial_statistics[i].name[10+NUM_TACTILE_SIGNALS+1+(j*3)].assign(std::string("l_pad_force_midle_row_") + usc_utilities::getString(j));
            accumulated_trial_statistics[i].name[10+NUM_TACTILE_SIGNALS+2+(j*3)].assign(std::string("l_pad_force_right_row_") + usc_utilities::getString(j));
        }
    }
}

bool GripperStatesRecorder::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
{
    vector_of_accumulated_trial_statistics.clear();
    std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics;
    ROS_VERIFY(accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
    setMessageNames(accumulated_trial_statistics);
    vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
    return true;
}


}
