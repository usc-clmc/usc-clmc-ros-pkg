/*
 * eigen_to_tf.h
 *
 *  Created on: Sept 24, 2013
 *      Author: manuel
 */

#ifndef EIGEN_TO_TF_H_
#define EIGEN_TO_TF_H_

#include <tf/transform_datatypes.h>
#include <Eigen/Dense>


namespace conversions
{
template<typename T> void convert(
    const Eigen::Quaternion<T>& eigen,
    tf::Quaternion& tf)
{
  tf = tf::Quaternion(eigen.x(), eigen.y(), eigen.z(), eigen.w());
}

template<typename T> void convert(
    const Eigen::Matrix<T,3,1>& eigen,
    tf::Vector3& tf)
{
  for (size_t col = 0; col < 3; col++)
    tf[col] = eigen(col);
}

template<typename T> void convert(
    const Eigen::Matrix<T, 4, 4>& eigen,
    tf::Transform& tf)
{
  tf.setIdentity();
  for (size_t col = 0; col < 3; col++)
    for (size_t row = 0; row < 3; row++)
      tf.getBasis()[col][row] = eigen(col, row);

  for (size_t col = 0; col < 3; col++)
     tf.getOrigin()[col] = eigen(col, 3);
}


}
#endif /* EIGEN_TO_TF_H_ */
