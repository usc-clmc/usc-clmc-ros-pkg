/*
 * conversions.h
 *
 *  Created on: Jun 17, 2011
 *      Author: kalakris
 */

#ifndef ROS_TO_KDL_H_
#define ROS_TO_KDL_H_

#include <kdl/frames.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace conversions
{

static void rosQuaternionToKdlRotation(const geometry_msgs::Quaternion& quaternion,
                                KDL::Rotation& rotation);
static void rosPointToKdlVector(const geometry_msgs::Point& point,
                                KDL::Vector& vector);
static void rosPoseToKdlFrame(const geometry_msgs::Pose& pose,
                              KDL::Frame& frame);

// inline implementations

static inline void rosQuaternionToKdlRotation(const geometry_msgs::Quaternion& quaternion,
                                KDL::Rotation& rotation)
{
  rotation = KDL::Rotation::Quaternion(quaternion.x,
                                       quaternion.y,
                                       quaternion.z,
                                       quaternion.w);
}

static inline void rosPointToKdlVector(const geometry_msgs::Point& point,
                                KDL::Vector& vector)
{
  vector.x(point.x);
  vector.y(point.y);
  vector.z(point.z);
}

static inline void rosPoseToKdlFrame(const geometry_msgs::Pose& pose,
                              KDL::Frame& frame)
{
  rosPointToKdlVector(pose.position, frame.p);
  rosQuaternionToKdlRotation(pose.orientation, frame.M);
}

}

#endif /* CONVERSIONS_H_ */
