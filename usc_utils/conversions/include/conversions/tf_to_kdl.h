/*
 * tf_to_kdl.h
 *
 *  Created on: Jan 11, 2013
 *      Author: kalakris
 */

#ifndef TF_TO_KDL_H_
#define TF_TO_KDL_H_

#include <conversions/tf_to_ros.h>
#include <conversions/ros_to_kdl.h>

namespace conversions
{

static void convert(const tf::Quaternion& quaternion,
                    KDL::Rotation& rotation);
static void convert(const tf::Point& point,
                    KDL::Vector& vector);
static void convert(const tf::Pose& pose,
                    KDL::Frame& frame);

// inline implementations

static inline void convert(const tf::Quaternion& quaternion,
                    KDL::Rotation& rotation)
{
  geometry_msgs::Quaternion ros_quat;
  convert(quaternion, ros_quat);
  convert(ros_quat, rotation);
}

static inline void convert(const tf::Point& point,
                    KDL::Vector& vector)
{
  geometry_msgs::Point ros_point;
  convert(point, ros_point);
  convert(ros_point, vector);
}

static inline void convert(const tf::Pose& pose,
                    KDL::Frame& frame)
{
  geometry_msgs::Pose ros_pose;
  convert(pose, ros_pose);
  convert(ros_pose, frame);
}

}



#endif /* TF_TO_KDL_H_ */
