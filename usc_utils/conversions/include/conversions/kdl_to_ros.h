/*
 * conversions.h
 *
 *  Created on: Jun 17, 2011
 *      Author: kalakris
 */

#ifndef KDL_TO_ROS_H_
#define KDL_TO_ROS_H_

#include <kdl/frames.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace conversions
{

static void convert(const KDL::Rotation& rotation,
                    geometry_msgs::Quaternion& quaternion);
static void convert(const KDL::Vector& vector,
                    geometry_msgs::Point& point);
static void convert(const KDL::Frame& frame,
                    geometry_msgs::Pose& pose);

// old names kept for backwards compatibility
static void kdlRotationToRosQuaternion(const KDL::Rotation& rotation,
                                       geometry_msgs::Quaternion& quaternion);
static void kdlVectorToRosPoint(const KDL::Vector& vector,
                                geometry_msgs::Point& point);
static void kdlFrameToRosPose(const KDL::Frame& frame,
                              geometry_msgs::Pose& pose);

// inline implementations

static inline void convert(const KDL::Rotation& rotation,
                    geometry_msgs::Quaternion& quaternion)
{
  rotation.GetQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}

static inline void convert(const KDL::Vector& vector,
                    geometry_msgs::Point& point)
{
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
}

static inline void convert(const KDL::Frame& frame,
                    geometry_msgs::Pose& pose)
{
  convert(frame.M, pose.orientation);
  convert(frame.p, pose.position);
}

static inline void kdlRotationToRosQuaternion(const KDL::Rotation& rotation,
                                       geometry_msgs::Quaternion& quaternion)
{
  convert(rotation, quaternion);
}

static inline void kdlVectorToRosPoint(const KDL::Vector& vector,
                                geometry_msgs::Point& point)
{
  convert(vector, point);
}

static inline void kdlFrameToRosPose(const KDL::Frame& frame,
                              geometry_msgs::Pose& pose)
{
  convert(frame, pose);
}

}

#endif /* KDL_TO_ROS_H_ */
