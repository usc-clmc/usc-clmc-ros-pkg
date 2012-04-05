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

 \file    imu_converter_node.cpp

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

// system includes
#include <sstream>

// ros includes
#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>

#include <kdl/frames.hpp>
#include <angles/angles.h>

#include <sensor_msgs/Imu.h>

class IMUConverterNode
{
public:

    IMUConverterNode(ros::NodeHandle node_handle);
    ~IMUConverterNode(){};

private:

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_in);
    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    ros::Publisher cube_publisher_;
    int loop_count_;
    int publishing_rate_;

};

IMUConverterNode::IMUConverterNode(ros::NodeHandle node_handle)
{
    node_handle_ = node_handle;
    subscriber_ = node_handle_.subscribe("/imu/data", 1000, &IMUConverterNode::imuCallback, this);
    publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(std::string("/box_pose"), 100);
    cube_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(std::string("/box"), 100);
    loop_count_ = 0;
    publishing_rate_ = 20;
}

void IMUConverterNode::imuCallback (const sensor_msgs::Imu::ConstPtr& imu_in)
{
    loop_count_ ++;
    if((loop_count_ % publishing_rate_) == 0)
    {
        geometry_msgs::PoseStamped box_pose;
        box_pose.header.frame_id = imu_in->header.frame_id;
        box_pose.header.stamp = ros::Time::now();
        box_pose.pose.orientation = imu_in->orientation;
        publisher_.publish(box_pose);

        visualization_msgs::Marker cube_msg;
        cube_msg.header.frame_id = std::string("/") + imu_in->header.frame_id;
        cube_msg.header.stamp = ros::Time::now();
        cube_msg.ns = "ChopStickTaskBox";
        cube_msg.type = visualization_msgs::Marker::CUBE;
        cube_msg.action = visualization_msgs::Marker::ADD;
        cube_msg.id = 13;

        cube_msg.scale.x = 2.0 * 0.0936;
        cube_msg.scale.y = 2.0 * 0.084;
        cube_msg.scale.z = 2.0 * 0.0566;

        cube_msg.pose.orientation = imu_in->orientation;

        cube_msg.color.r = 0.74;
        cube_msg.color.g = 0.74;
        cube_msg.color.b = 0.74;
        cube_msg.color.a = 1.0;
        cube_publisher_.publish(cube_msg);

        double roll, pitch, yaw;
        KDL::Rotation box_orientation = KDL::Rotation::Quaternion(imu_in->orientation.x,
                                                                  imu_in->orientation.y,
                                                                  imu_in->orientation.z,
                                                                  imu_in->orientation.w);
        box_orientation.GetRPY(roll, pitch, yaw);

        double normalized_roll = angles::normalize_angle_positive(roll);
        double normalized_pitch = angles::normalize_angle_positive(pitch);
        double normalized_yaw = angles::normalize_angle_positive(yaw);

        if(normalized_roll > 5)
        {
            normalized_roll = 0.0;
        }

        ROS_INFO(">> %1.2f\t %1.2f \t %1.2f | %1.2f \t %1.2f \t %1.2f | %1.2f \t %1.2f \t %1.2f",
                 normalized_roll, normalized_pitch, normalized_yaw,
                     imu_in->linear_acceleration.x, imu_in->linear_acceleration.y, imu_in->linear_acceleration.z,
                     imu_in->angular_velocity.x, imu_in->angular_velocity.y, imu_in->angular_velocity.z);

        const double FONT_SIZE = 0.08;
        visualization_msgs::Marker text_msg;
        text_msg.header.frame_id = std::string("/imu");
        text_msg.header.stamp = ros::Time::now();
        text_msg.ns = "ChopStickTaskBoxText";
        text_msg.id = 742;
        text_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_msg.action = visualization_msgs::Marker::ADD;
        text_msg.pose.position.x = 0.0;
        text_msg.pose.position.y = - 0.5;
        text_msg.pose.position.z = - 0.25;
        text_msg.pose.orientation.x = 0.0;
        text_msg.pose.orientation.y = 0.0;
        text_msg.pose.orientation.z = 0.0;
        text_msg.pose.orientation.w = 1.0;
        text_msg.scale.x = FONT_SIZE;
        text_msg.scale.y = FONT_SIZE;
        text_msg.scale.z = FONT_SIZE;

        text_msg.lifetime = ros::Duration();
        text_msg.color.a = 1.0;

        std::stringstream ss;
        int precision = 2;
        ss.precision(precision);
        ss << std::fixed;

        ss << normalized_roll;
        text_msg.text = std::string("Box Orientation: ") + ss.str();
        text_msg.color.r = 1.0;
        text_msg.color.g = 1.0;
        text_msg.color.b = 1.0;

        cube_publisher_.publish(text_msg);
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_converter_node");
    ros::NodeHandle node_handle("~");

    IMUConverterNode converter(node_handle);
    ros::spin();
    return 1;
}


