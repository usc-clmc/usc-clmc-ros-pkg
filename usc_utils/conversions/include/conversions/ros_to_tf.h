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

static void convert(const geometry_msgs::Pose& ros_pose,
                    tf::Transform& tf_transform);

// inline implementations
static inline void convert(const geometry_msgs::Pose& ros_pose,
                    tf::Transform& tf_transform)
{
  tf::poseMsgToTF(ros_pose, tf_transform);
}

}


#endif /* CONVERSIONS_ROS_TO_TF_H_ */
