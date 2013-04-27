/*
 * ros_to_tf.h
 *
 *  Created on: Mar 5, 2012
 *      Author: kalakris
 */

#ifndef CONVERSIONS_ROS_TO_TF_H_
#define CONVERSIONS_ROS_TO_TF_H_

#include <tf/transform_datatypes.h>

namespace conversions
{

static void convert(const geometry_msgs::Quaternion& ros_quat,
                    tf::Quaternion& tf_quat);
static void convert(const geometry_msgs::Point& ros_point,
                    tf::Point& tf_point);
static void convert(const geometry_msgs::Vector3& ros_vector,
                    tf::Vector3& tf_vector);
static void convert(const geometry_msgs::Pose& ros_pose,
                    tf::Transform& tf_transform);
static void convert(const geometry_msgs::Transform& ros_transform,
                    tf::Transform& tf_transform);

static tf::Pose convertToTFPose(const geometry_msgs::Pose& ros_pose);

// inline implementations
static inline void convert(const geometry_msgs::Pose& ros_pose,
                    tf::Transform& tf_transform)
{
  tf::poseMsgToTF(ros_pose, tf_transform);
}

static inline void convert(const geometry_msgs::Transform& ros_transform,
                    tf::Transform& tf_transform)
{
  tf::transformMsgToTF(ros_transform, tf_transform);
}

static inline void convert(const geometry_msgs::Quaternion& ros_quat,
                    tf::Quaternion& tf_quat)
{
  tf::quaternionMsgToTF(ros_quat, tf_quat);
}

static inline void convert(const geometry_msgs::Point& ros_point,
                    tf::Point& tf_point)
{
  tf::pointMsgToTF(ros_point, tf_point);
}

static inline void convert(const geometry_msgs::Vector3& ros_vector,
                    tf::Vector3& tf_vector)
{
  tf::vector3MsgToTF(ros_vector, tf_vector);
}

static inline tf::Pose convertToTFPose(const geometry_msgs::Pose& ros_pose)
{
  tf::Pose tf_pose;
  convert(ros_pose, tf_pose);
  return tf_pose;
}

}


#endif /* CONVERSIONS_ROS_TO_TF_H_ */
