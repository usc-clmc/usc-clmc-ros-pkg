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

 \file    joint_states_recorder.cpp

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

// system includes

// ros includes
#include <usc_utilities/bspline.h>
#include <usc_utilities/assert.h>

// local includes
#include <task_recorder/joint_states_recorder.h>
#include <task_recorder/task_recorder_utilities.h>

namespace task_recorder
{

bool JointStatesRecorder::initialize(ros::NodeHandle& node_handle,
                                     const std::string& topic_name)
{
  return initializeBase(node_handle, topic_name);
}

bool JointStatesRecorder::filterAndCrop(const ros::Time& start_time,
                                        const ros::Time& end_time,
                                        int num_samples,
                                        std::vector<sensor_msgs::JointState>& filter_and_cropped_joint_states,
                                        std::vector<std::string>& message_names,
                                        std::vector<ros::Time>& times,
                                        std::vector<double>& data)
{

  int num_joint_states = recorder_io_.messages_.size();
  if (num_joint_states == 0)
  {
    ROS_ERROR("Zero joint states have been logged.");
    return false;
  }

  // figure out when our data starts and ends
  ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
  ros::Time our_end_time = recorder_io_.messages_[num_joint_states - 1].header.stamp;
  int index = 0;
  while (our_end_time.toSec() < 1e-6)
  {
    index++;
    our_end_time = recorder_io_.messages_[num_joint_states - (1 + index)].header.stamp;
    ROS_WARN("Invalid time stamps in joint states");
  }

  if (our_start_time > start_time || our_end_time < end_time)
  {
    ROS_ERROR("Requested times have not been recorded!");
    ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
    ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
    return false;
  }

  // first crop
  ROS_VERIFY(crop<sensor_msgs::JointState>(recorder_io_.messages_, start_time, end_time));
  // then remove duplicates
  ROS_VERIFY(removeDuplicates<sensor_msgs::JointState>(recorder_io_.messages_));

  // ROS_INFO("Writing raw joint states data...");
  ROS_VERIFY(recorder_io_.writeRawData());
  // ROS_INFO("done");

  // fit bspline and resample the position and effort trajectories and compute the velocities
  ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples,
          filter_and_cropped_joint_states, message_names));
  ROS_ASSERT(static_cast<int>(filter_and_cropped_joint_states.size()) == num_samples);

  recorder_io_.messages_.clear();
  recorder_io_.messages_ = filter_and_cropped_joint_states;
  return true;
}

bool JointStatesRecorder::transformMessages(sensor_msgs::JointState& joint_state)
{
  return true;
}

bool JointStatesRecorder::resample(std::vector<sensor_msgs::JointState>& joint_states,
                                   const ros::Time& start_time,
                                   const ros::Time& end_time,
                                   const int num_samples,
                                   std::vector<sensor_msgs::JointState>& resampled_joint_states,
                                   std::vector<std::string> joint_names)
{

  ROS_VERIFY(!joint_states.empty());
  ROS_VERIFY(!joint_names.empty());

  int num_joint_states = static_cast<int> (joint_states.size());
  int num_joints = static_cast<int> (joint_names.size());

  // compute mean dt of the provided time stamps
  double dts[num_joint_states - 1];
  double mean_dt = 0.0;

  std::vector<double> input_vector(num_joint_states);
  input_vector[0] = joint_states[0].header.stamp.toSec();
  for (int i = 0; i < num_joint_states - 1; i++)
  {
    dts[i] = joint_states[i + 1].header.stamp.toSec() - joint_states[i].header.stamp.toSec();
    mean_dt += dts[i];
    input_vector[i + 1] = input_vector[i] + dts[i];
  }
  mean_dt /= static_cast<double> (num_joint_states - 1);

  ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

  double wave_length = interval.toSec() * 2;

  resampled_joint_states.clear();
  std::string frame_id = joint_states[0].header.frame_id;
  std::vector<double> input_querry(num_samples);
  for (int i = 0; i < num_samples; i++)
  {
    sensor_msgs::JointState js;
    js.header.frame_id = frame_id;
    js.header.seq = i;
    js.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
    js.name = joint_names;
    js.velocity.resize(num_joints);
    js.position.resize(num_joints);
    js.effort.resize(num_joints);
    input_querry[i] = js.header.stamp.toSec();
    resampled_joint_states.push_back(js);
  }

  //ROS_VERIFY(position_accumulator_.initialize(num_joints, num_samples));
  //ROS_VERIFY(velocity_accumulator_.initialize(num_joints, num_samples));
  //ROS_VERIFY(effort_accumulator_.initialize(num_joints, num_samples));

  for (int i = 0; i < static_cast<int> (joint_states[0].name.size()); ++i)
  {
    for (int n = 0; n < num_joints; ++n)
    {
      if (joint_names[n].compare(joint_states[0].name[i]) == 0)
      {
        std::vector<double> position_resampled;
        std::vector<double> effort_resampled;
        std::vector<double> velocity_resampled;
        std::vector<double> position_target_vector;
        std::vector<double> effort_target_vector;

        for (int j = 0; j < num_joint_states; ++j)
        {
          position_target_vector.push_back(joint_states[j].position[i]);
          effort_target_vector.push_back(joint_states[j].effort[i]);
        }
        if (!usc_utilities::resample(input_vector, position_target_vector, wave_length, input_querry, position_resampled, false))
        {
          ROS_ERROR("Could not rescale position trajectory, splining failed.");
          return false;
        }
        if (!usc_utilities::resample(input_vector, position_target_vector, wave_length, input_querry, velocity_resampled, true))
        {
          ROS_ERROR("Could not rescale velocity trajectory, splining failed.");
          return false;
        }
        if (!usc_utilities::resample(input_vector, effort_target_vector, wave_length, input_querry, effort_resampled, false))
        {
          ROS_ERROR("Could not rescale effort trajectory, splining failed.");
          return false;
        }

        for (int j = 0; j < num_samples; ++j)
        {
          resampled_joint_states[j].position[n] = position_resampled[j];
          resampled_joint_states[j].velocity[n] = velocity_resampled[j];
          resampled_joint_states[j].effort[n] = effort_resampled[j];
        }
        //ROS_VERIFY(position_accumulator_.add(n, position_resampled));
        //ROS_VERIFY(velocity_accumulator_.add(n, velocity_resampled));
        //ROS_VERIFY(effort_accumulator_.add(n, effort_resampled));
      }
    }
  }
  return true;
}

void JointStatesRecorder::getSignalNames(const int signal_index,
                                         std::string& signal_name)
{
  signal_name.assign("");
  if (signal_index == 0)
  {
    signal_name.assign("_position_");
  }
  else if (signal_index == 1)
  {
    signal_name.assign("_velocity_");
  }
  else if (signal_index == 2)
  {
    signal_name.assign("_effort_");
  }
}

void JointStatesRecorder::setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics)
{
  for (int i = 0; i < static_cast<int> (accumulated_trial_statistics.size()); ++i)
  {
    accumulated_trial_statistics[i].name = recorder_io_.messages_[0].name;
  }
}

bool JointStatesRecorder::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
{
  vector_of_accumulated_trial_statistics.clear();
  std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics;
  ROS_VERIFY(position_accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
  setMessageNames(accumulated_trial_statistics);
  vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
  ROS_VERIFY(velocity_accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
  setMessageNames(accumulated_trial_statistics);
  vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
  ROS_VERIFY(effort_accumulator_.getAccumulatedTrialStatistics(accumulated_trial_statistics));
  setMessageNames(accumulated_trial_statistics);
  vector_of_accumulated_trial_statistics.push_back(accumulated_trial_statistics);
  return true;
}

void JointStatesRecorder::getVariables(const sensor_msgs::JointState& message, std::vector<double>& variables)
{
  // TODO!! (not used because we implemented filterAndCrop internally)
}

void JointStatesRecorder::setVariables(sensor_msgs::JointState& message, const std::vector<double>& variables)
{
  // TODO!! (not used because we implemented filterAndCrop internally)
}

int JointStatesRecorder::getNumVariables()
{
  // TODO!! (not used because we implemented filterAndCrop internally)
  return 0;
}

}
