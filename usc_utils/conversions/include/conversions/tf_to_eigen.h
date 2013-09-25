/*
 * tf_to_eigen.h
 *
 *  Created on: Sept 24, 2013
 *      Author: manuel
 */

#ifndef TF_TO_EIGEN_H_
#define TF_TO_EIGEN_H_

#include <tf/transform_datatypes.h>
#include <Eigen/Dense>


namespace conversions
{
template<typename T>
void convert(const tf::Quaternion& tf,
             Eigen::Quaternion<T>& eigen)
{
  eigen = Eigen::Quaternion<T>(tf.w(), tf.x(), tf.y(), tf.z());
}

template<typename T>
void convert(const tf::Vector3& tf,
             Eigen::Matrix<T,3,1>& eigen)
{
  for (size_t col = 0; col < 3; col++)
    eigen(col) = tf[col];
}

template<typename T>
void convert(const tf::Transform& tf,
             Eigen::Matrix<T, 4, 4>& eigen)
{
  for (size_t col = 0; col < 3; col++)
    for (size_t row = 0; row < 3; row++)
      eigen(col, row) = tf.getBasis()[col][row];

  for (size_t col = 0; col < 3; col++)
    eigen(col, 3) = tf.getOrigin()[col];

  eigen.bottomLeftCorner(1, 3) = Eigen::Matrix<T, 1, 3>::Zero();
  eigen(3,3) = 1;
}


}
#endif /* TF_TO_EIGEN_H_ */
