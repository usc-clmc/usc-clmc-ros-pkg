/*
 * tf_to_ros.h
 *
 *  Created on: Mar 5, 2012
 *      Author: kalakris
 */

#ifndef CONVERSIONS_TF_TO_ROS_H_
#define CONVERSIONS_TF_TO_ROS_H_

#include <tf/transform_datatypes.h>

namespace conversions
{

static void convert(const tf::Transform& tf_transform,
                    geometry_msgs::Pose& ros_pose);

static void convert(const tf::Vector3& tf_vector3,
                    geometry_msgs::Point& ros_point);

static void convert(const tf::Quaternion& tf_quat,
                    geometry_msgs::Quaternion& ros_quat);

static geometry_msgs::Pose convertToROSPose(const tf::Pose& tf_pose);

// inline implementations
static inline void convert(const tf::Transform& tf_transform,
                           geometry_msgs::Pose& ros_pose)
{
  tf::poseTFToMsg(tf_transform, ros_pose);
}

static inline void convert(const tf::Vector3& tf_vector3,
                           geometry_msgs::Point& ros_point)
{
  tf::pointTFToMsg(tf_vector3, ros_point);
}

static inline void convert(const tf::Quaternion& tf_quat,
                    geometry_msgs::Quaternion& ros_quat)
{
  tf::quaternionTFToMsg(tf_quat, ros_quat);
}

static inline geometry_msgs::Pose convertToROSPose(const tf::Pose& tf_pose)
{
  geometry_msgs::Pose ros_pose;
  convert(tf_pose, ros_pose);
  return ros_pose;
}

}


#endif /* CONVERSIONS_ROS_TO_TF_H_ */
