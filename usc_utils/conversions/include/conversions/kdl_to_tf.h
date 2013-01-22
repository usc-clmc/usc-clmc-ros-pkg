/*
 * kdl_to_tf.h
 *
 *  Created on: Jan 11, 2013
 *      Author: kalakris
 */

#ifndef KDL_TO_TF_H_
#define KDL_TO_TF_H_

#include <kdl/frames.hpp>
#include <tf/transform_datatypes.h>
#include <conversions/kdl_to_ros.h>
#include <conversions/ros_to_tf.h>

namespace conversions
{

static void convert(const KDL::Rotation& rotation,
                    tf::Quaternion& quaternion);
static void convert(const KDL::Vector& vector,
                    tf::Point& point);
static void convert(const KDL::Frame& frame,
                    tf::Pose& pose);

// inline implementations

static void inline convert(const KDL::Rotation& rotation,
                    tf::Quaternion& quaternion)
{
  geometry_msgs::Quaternion ros_quat;
  convert(rotation, ros_quat);
  convert(ros_quat, quaternion);
}

static void inline convert(const KDL::Vector& vector,
                    tf::Point& point)
{
  geometry_msgs::Point ros_point;
  convert(vector, ros_point);
  convert(ros_point, point);
}

static void inline convert(const KDL::Frame& frame,
                    tf::Pose& pose)
{
  geometry_msgs::Pose ros_pose;
  convert(frame, ros_pose);
  convert(ros_pose, pose);
}

}

#endif /* KDL_TO_TF_H_ */
