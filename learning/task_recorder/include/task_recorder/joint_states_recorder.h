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

 \file    joint_states_recorder.h

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

#ifndef TASK_RECORDER1_JOINT_STATES_RECORDER_H_
#define TASK_RECORDER1_JOINT_STATES_RECORDER_H_

// system includes
#include <vector>

// ros includes
#include <sensor_msgs/JointState.h>

// local includes
#include <task_recorder/task_recorder.h>
#include <task_recorder/accumulator.h>

#include <task_recorder/StartRecording.h>
#include <task_recorder/StopRecordingJointStates.h>

namespace task_recorder
{

class JointStatesRecorder : public TaskRecorder<sensor_msgs::JointState, task_recorder::StopRecordingJointStates>
{

public:

  /*! Constructor
   */
  JointStatesRecorder() {};
  virtual ~JointStatesRecorder() {};

  /*!
   * @param node_handle
   * @param topic_name
   * @return
   */
  bool initialize(ros::NodeHandle& node_handle,
                  const std::string& topic_name);

  /*!
   * @param start_time
   * @param end_time
   * @param movement_duration
   * @param num_samples
   * @param joint_states
   * @param message_names
   * @param times
   * @param data
   * @return
   */
  bool filterAndCrop(const ros::Time& start_time,
                     const ros::Time& end_time,
                     int num_samples,
                     std::vector<sensor_msgs::JointState>& joint_states,
                     std::vector<std::string>& message_names,
                     std::vector<ros::Time>& times,
                     std::vector<double>& data);

  bool transformMessages(sensor_msgs::JointState& joint_state);

  /*!
   * @param trial_statistics
   * @return
   */
  bool getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics);

  /*!
   * @param signal_index
   * @param signal_name
   */
  void getSignalNames(const int signal_index,
                      std::string& signal_name);

  /*!
   * @param accumulated_trial_statistics
   */
  void setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics);

  void getVariables(const sensor_msgs::JointState& message, std::vector<double>& variables);
  void setVariables(sensor_msgs::JointState& message, const std::vector<double>& variables);
  int getNumVariables();

private:

  bool resample(std::vector<sensor_msgs::JointState>& joint_states,
                const ros::Time& start_time,
                const ros::Time& end_time,
                const int num_samples,
                std::vector<sensor_msgs::JointState>& resampled_joint_states,
                std::vector<std::string> joint_names);

  Accumulator position_accumulator_;
  Accumulator velocity_accumulator_;
  Accumulator effort_accumulator_;

};

}

#endif /* JOINT_STATES_RECORDER_H_ */
