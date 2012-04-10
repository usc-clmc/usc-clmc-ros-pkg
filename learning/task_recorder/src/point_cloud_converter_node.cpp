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

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

class PointCloudConverter
{
public:

    PointCloudConverter(ros::NodeHandle node_handle);
    ~PointCloudConverter(){};

private:

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

    ros::NodeHandle node_handle_;

    laser_geometry::LaserProjection projector_;

    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
};

void PointCloudConverter::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan_in, cloud);
    publisher_.publish(cloud);
}

PointCloudConverter::PointCloudConverter(ros::NodeHandle node_handle)
{
    node_handle_ = node_handle;
    subscriber_ = node_handle_.subscribe("/scan", 1000, &PointCloudConverter::scanCallback, this);
    publisher_ = node_handle_.advertise<sensor_msgs::PointCloud>(std::string("/ball_point_cloud"), 1000);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_conver_node");
    ros::NodeHandle node_handle("~");

    PointCloudConverter converter(node_handle);
    ros::spin();
    return 1;
}


