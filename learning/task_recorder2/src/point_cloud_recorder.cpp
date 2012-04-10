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

 \file    point_cloud_recorder.cpp

 \author  Peter Pastor
 \date    Aug 06, 2010

 **********************************************************************/

// system includes
#include <sstream>

// ros includes
#include <usc_utilities/bspline.h>
#include <usc_utilities/assert.h>

#include <geometry_msgs/PointStamped.h>


// local includes
#include <task_recorder2/point_cloud_recorder.h>

namespace task_recorder2
{

PointCloudRecorder::PointCloudRecorder(ros::NodeHandle node_handle) : TaskRecorder<sensor_msgs::PointCloud, task_recorder2::StopRecordingPointCloud>(node_handle)
{
}

PointCloudRecorder::~PointCloudRecorder()
{
}

bool PointCloudRecorder::initialize(const std::string& topic_name)
{
    return initialize(topic_name);
}

bool PointCloudRecorder::filterAndCrop(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration, int num_samples,
                                        std::vector<sensor_msgs::PointCloud>& filtered_and_cropped_point_cloud, std::vector<std::string>& message_names,
                                        std::vector<ros::Time>& times, std::vector<double>& data)
{

    int num_point_cloud = recorder_io_.messages_.size();
    if (num_point_cloud == 0)
    {
        ROS_ERROR("Zero point_clouds have been logged.");
        return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
    ros::Time our_end_time = recorder_io_.messages_[num_point_cloud - 1].header.stamp;
    int index = 0;
    while (our_end_time.toSec() < 1e-6)
    {
        index++;
        our_end_time = recorder_io_.messages_[num_point_cloud - (1+index)].header.stamp;
    }

    if (our_start_time > start_time || our_end_time < end_time)
    {
        ROS_ERROR("Requested times have not been recorded!");
        ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
        ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
        return false;
    }

    ros::Time detection_time = ros::Time(0);
    double offset;
    ROS_VERIFY(detectBall(start_time, end_time, movement_duration, detection_time, offset));

    times.clear();
    data.clear();
    filtered_and_cropped_point_cloud.clear();
    if(detection_time > start_time && detection_time < end_time)
    {
        sensor_msgs::PointCloud fake_point_cloud;
        fake_point_cloud.header.frame_id = std::string("/laser");
        fake_point_cloud.header.stamp = detection_time;
        fake_point_cloud.header.seq = 1;
        fake_point_cloud.points.clear();
        geometry_msgs::Point32 point;
        point.x = offset;
        point.y = offset;
        point.z = offset;
        fake_point_cloud.points.push_back(point);
        filtered_and_cropped_point_cloud.push_back(fake_point_cloud);
        times.push_back(detection_time);
        data.push_back(offset);
    }

    // fit bspline and resample the position and effort trajectories and compute the velocities
    // ROS_VERIFY(resample(recorder_io_.messages_, start_time, end_time, num_samples, filtered_and_cropped_point_cloud));
    // ROS_ASSERT(static_cast<int>(filtered_and_cropped_point_cloud.size()) == num_samples);

    recorder_io_.messages_.clear();
    recorder_io_.messages_ = filtered_and_cropped_point_cloud;
    return true;
}

bool PointCloudRecorder::detectBall(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration, ros::Time& detection_time, double& offset)
{

    bool ball_detected = false;

    double average_depth = 0;
    for (int i=0; i<static_cast<int>(recorder_io_.messages_.size()); ++i)
    {
        if((recorder_io_.messages_[i].header.stamp >= start_time)
                && (recorder_io_.messages_[i].header.stamp < start_time + ros::Duration(movement_duration)))
        {
            int num_points = static_cast<int>(recorder_io_.messages_[i].points.size());
            for (int j=0; j<num_points; ++j)
            {
                average_depth += recorder_io_.messages_[i].points[j].x;
            }
            average_depth /= num_points;
        }
    }

    // ROS_INFO("Avg depth is %f", average_depth);
    for (int i=0; i<static_cast<int>(recorder_io_.messages_.size()) && !ball_detected; ++i)
    {
        if((recorder_io_.messages_[i].header.stamp >= start_time + ros::Duration(movement_duration))
                && (recorder_io_.messages_[i].header.stamp < end_time))
        {
            int num_points = static_cast<int>(recorder_io_.messages_[i].points.size());
            for (int j=0; j<num_points; ++j)
            {
                double ball_threshold_height = 0.04;
                double ball_threshold = average_depth - ball_threshold_height;
                if(recorder_io_.messages_[i].points[j].x < ball_threshold)
                {
                    double min_depth = average_depth;
                    for (int n=0; n<num_points; ++n)
                    {
                        if(recorder_io_.messages_[i].points[n].x < min_depth)
                        {
                            min_depth = recorder_io_.messages_[i].points[n].x;
                            offset = recorder_io_.messages_[i].points[n].y;
                            // ROS_INFO("smallest is: %f", offset);
                        }
                    }
                    detection_time = recorder_io_.messages_[i].header.stamp;
                    ball_detected = true;
                    // ROS_INFO("Ball detected at %f.", offset);
                    break;
                }
            }
        }
    }

//    std::string file_name = std::string("/tmp/ball_point_cloud.bag");
//    ROS_INFO_STREAM("Writing to file: " << file_name);
//    try
//    {
//        rosbag::Bag bag;
//        bag.open(file_name, rosbag::bagmode::Write);
//        for (int i = 0; i < static_cast<int> (recorder_io_.messages_.size()); ++i)
//        {
//            if (recorder_io_.messages_[i].header.stamp > ros::TIME_MIN)
//            {
//                bag.write(std::string("/ball_point_cloud"), recorder_io_.messages_[i].header.stamp, recorder_io_.messages_[i]);
//            }
//        }
//        bag.close();
//    }
//    catch (rosbag::BagIOException ex)
//    {
//        ROS_ERROR("Problem when writing to bag file named %s.", file_name.c_str());
//        return false;
//    }
    return true;
}

bool PointCloudRecorder::transformMessages(sensor_msgs::PointCloud& point_cloud_state)
{
    return true;
}

bool PointCloudRecorder::getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
{
    return true;
}

void PointCloudRecorder::setMessageNames(std::vector<task_recorder2_msgs::AccumulatedTrialStatistics>& trial_statistics)
{

}

//bool PointCloudRecorder::resample(std::vector<sensor_msgs::PointCloud>& point_cloud,
//                                           const ros::Time& start_time, const ros::Time& end_time, const int num_samples,
//                                           std::vector<sensor_msgs::PointCloud>& resampled_point_cloud)
//{
//
//    ROS_VERIFY(!point_cloud.empty());
//    ROS_VERIFY(removeDuplicates<sensor_msgs::PointCloud>(point_cloud));
//
//    int num_point_cloud = static_cast<int> (point_cloud.size());
//
//    // compute mean dt of the provided time stamps
//    double dts[num_point_cloud - 1];
//    double mean_dt = 0.0;
//
//    std::vector<double> input_vector(num_point_cloud);
//    input_vector[0] = point_cloud[0].header.stamp.toSec();
//    for (int i = 0; i < num_point_cloud - 1; i++)
//    {
//        dts[i] = point_cloud[i + 1].header.stamp.toSec() - point_cloud[i].header.stamp.toSec();
//        mean_dt += dts[i];
//        input_vector[i + 1] = input_vector[i] + dts[i];
//    }
//    mean_dt /= static_cast<double> (num_point_cloud - 1);
//
//    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));
//
//    double wave_length = interval.toSec() * static_cast<double>(2.0);
//
//    resampled_point_cloud.clear();
//    std::string frame_id = point_cloud[0].header.frame_id;
//    std::vector<double> input_querry(num_samples);
//    for (int i = 0; i < num_samples; i++)
//    {
//        sensor_msgs::PointCloud ls;
//        ls.header.frame_id = frame_id;
//        ls.header.seq = i;
//        ls.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
//        ls.samples.resize(1);
//        input_querry[i] = ls.header.stamp.toSec();
//        resampled_point_cloud.push_back(ls);
//    }
//
//    std::vector<double> acceleration_target_vector_x;
//    std::vector<double> acceleration_target_vector_y;
//    std::vector<double> acceleration_target_vector_z;
//    std::vector<double> acceleration_vector_resampled_x;
//    std::vector<double> acceleration_vector_resampled_y;
//    std::vector<double> acceleration_vector_resampled_z;
//    for (int j = 0; j < num_point_cloud; ++j)
//    {
//        // ROS_INFO("x = %f", point_cloud[j].samples[0].x);
//        acceleration_target_vector_x.push_back(point_cloud[j].samples[0].x);
//        acceleration_target_vector_y.push_back(point_cloud[j].samples[0].y);
//        acceleration_target_vector_z.push_back(point_cloud[j].samples[0].z);
//    }
//
//    if(!usc_utilities::resample(input_vector, acceleration_target_vector_x, wave_length, input_querry, acceleration_vector_resampled_x, false))
//    {
//        ROS_ERROR("Could not rescale acceleration vector x, splining failed.");
//        return false;
//    }
//    if(!usc_utilities::resample(input_vector, acceleration_target_vector_y, wave_length, input_querry, acceleration_vector_resampled_y, false))
//    {
//        ROS_ERROR("Could not rescale acceleration vector y, splining failed.");
//        return false;
//    }
//    if(!usc_utilities::resample(input_vector, acceleration_target_vector_z, wave_length, input_querry, acceleration_vector_resampled_z, false))
//    {
//        ROS_ERROR("Could not rescale acceleration vector z, splining failed.");
//        return false;
//    }
//
//    for (int j = 0; j < num_samples; ++j)
//    {
//        resampled_point_cloud[j].samples[0].x = acceleration_vector_resampled_x[j];
//        resampled_point_cloud[j].samples[0].y = acceleration_vector_resampled_y[j];
//        resampled_point_cloud[j].samples[0].z = acceleration_vector_resampled_z[j];
//    }
//
//    ROS_VERIFY(accumulator_.add(0, acceleration_vector_resampled_x));
//    ROS_VERIFY(accumulator_.add(1, acceleration_vector_resampled_y));
//    ROS_VERIFY(accumulator_.add(2, acceleration_vector_resampled_z));
//
//    return true;
//}

void PointCloudRecorder::getSignalNames(const int signal_index, std::string& signal_name)
{
    signal_name.assign("");
}

//void PointCloudRecorder::setMessageNames(std::vector<task_recorder2_msgs::AccumulatedTrialStatistics>& accumulated_trial_statistics)
//{
//    for (int i=0; i<static_cast<int>(accumulated_trial_statistics.size()); ++i)
//    {
//        accumulated_trial_statistics[i].name.resize(NUM_ACCELEROMETER_SIGNALS);
//        accumulated_trial_statistics[i].name[0].assign(std::string("gripper_accelerometer_x"));
//        accumulated_trial_statistics[i].name[1].assign(std::string("gripper_accelerometer_y"));
//        accumulated_trial_statistics[i].name[2].assign(std::string("gripper_accelerometer_z"));
//    }
//}


}
