/*
 * kdl_to_eigen.h
 *
 *  Created on: Jun 17, 2011
 *      Author: kalakris
 */

#ifndef KDL_TO_EIGEN_H_
#define KDL_TO_EIGEN_H_

#include <kdl/frames.hpp>
#include <Eigen/Dense>

namespace conversions
{

// in the case of eigen, it helps to have specific function names

static void kdlVectorToEigenVector(const KDL::Vector& kdl_vector,
                                   Eigen::Vector3d& eigen_vector);
static void kdlRotationToEigenMatrix3d(const KDL::Rotation& kdl_rotation,
                                       Eigen::Matrix3d& eigen_matrix);

static void convert(const KDL::Vector& kdl_vector,
                                   Eigen::Vector3d& eigen_vector);
static void convert(const KDL::Rotation& kdl_rotation,
                                       Eigen::Matrix3d& eigen_matrix);

// inline functions:

static void convert(const KDL::Vector& kdl_vector,
                                   Eigen::Vector3d& eigen_vector)
{
  kdlVectorToEigenVector(kdl_vector, eigen_vector);
}

static void convert(const KDL::Rotation& kdl_rotation,
                                       Eigen::Matrix3d& eigen_matrix)
{
  kdlRotationToEigenMatrix3d(kdl_rotation, eigen_matrix);
}

static inline void kdlVectorToEigenVector(const KDL::Vector& kdl_vector,
                                   Eigen::Vector3d& eigen_vector)
{
  for (int i=0; i<3; ++i)
  {
    eigen_vector[i] = kdl_vector[i];
  }
}

static inline void kdlRotationToEigenMatrix3d(const KDL::Rotation& kdl_rotation,
                                       Eigen::Matrix3d& eigen_matrix)
{
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      eigen_matrix(i,j) = kdl_rotation(i,j);
}

}


#endif /* KDL_TO_EIGEN_H_ */
