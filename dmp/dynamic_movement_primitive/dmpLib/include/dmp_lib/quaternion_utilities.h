/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		quaternion_utilities.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Feb 13, 2011

 *********************************************************************/

#ifndef QUATERNION_UTILITIES_H_
#define QUATERNION_UTILITIES_H_

// system includes
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// local includes

namespace dmp_lib
{

/*!
 * @param quat_current
 * @param quat_desired
 * @param error
 * @return True on success, otherwise False
 */
void getQuatError(const Eigen::Vector4d& quat_current, const Eigen::Vector4d& quat_desired, Eigen::Vector3d& error);

/*!
 * @param angular_velocity
 * @param quatd
 * @param quat
 */
void getQuaternionVelocity(const Eigen::Vector4d& quat, const Eigen::Vector3d& angular_velocity, Eigen::Vector4d& quatd);

/*!
 * @param quat
 * @param quatd
 * @param angular_velocity
 * REAL-TIME REQUIREMENTS
 */
void getAngularVelocity(const Eigen::Vector4d& quat, const Eigen::Vector4d& quatd, Eigen::Vector3d& angular_velocity);

/*!
 * @param quat
 * @param quatd
 * @param quatdd
 * @param angular_acceleration
 * REAL-TIME REQUIREMENTS
 */
void getAngularAccelerations(const Eigen::Vector4d& quat, const Eigen::Vector4d& quatd, const Eigen::Vector4d& quatdd, Eigen::Vector3d& angular_acceleration);

/*!
 * @param quat
 * @param Q
 * REAL-TIME REQUIREMENTS
 */
void getQMatrix(const Eigen::Vector4d& quat,
                Eigen::Matrix<double, 4, 3>& Q);

// REAL-TIME REQUIREMENT
inline void getQuatError(const Eigen::Vector4d& quat_current, const Eigen::Vector4d& quat_desired, Eigen::Vector3d& error)
{
  double n1 = quat_current(0);
  Eigen::Vector3d q1 = quat_current.segment(1, 3);
  double n2 = quat_desired(0);
  Eigen::Vector3d q2 = quat_desired.segment(1, 3);
  error = n1*q2 - n2*q1 + q1.cross(q2);
}

inline void getQMatrix(const Eigen::Vector4d& quat,
                       Eigen::Matrix<double, 4, 3>& Q)
{
  Q(0, 0) = -quat(1);
  Q(0, 1) = -quat(2);
  Q(0, 2) = -quat(3);
  Q(1, 0) =  quat(0);
  Q(1, 1) =  quat(3);
  Q(1, 2) = -quat(2);
  Q(2, 0) = -quat(3);
  Q(2, 1) =  quat(0);
  Q(2, 2) =  quat(1);
  Q(3, 0) =  quat(2);
  Q(3, 1) = -quat(1);
  Q(3, 2) =  quat(0);
}

// REAL-TIME REQUIREMENT
inline void getQuaternionVelocity(const Eigen::Vector4d& quat, const Eigen::Vector3d& angular_velocity, Eigen::Vector4d& quatd)
{
  Eigen::Matrix<double, 4, 3> Q;
  getQMatrix(quat, Q);
  quatd = 0.5 * Q * angular_velocity;
}

// REAL-TIME REQUIREMENT
inline void getAngularVelocity(const Eigen::Vector4d& quat, const Eigen::Vector4d& quatd, Eigen::Vector3d& angular_velocity)
{
  Eigen::Matrix<double, 4, 3> Q;
  getQMatrix(quat, Q);
  angular_velocity = 2.0 * Q.transpose() * quatd;
}

// REAL-TIME REQUIREMENT
inline void getAngularAccelerations(const Eigen::Vector4d& quat, const Eigen::Vector4d& quatd, const Eigen::Vector4d& quatdd, Eigen::Vector3d& angular_acceleration)
{
  Eigen::Matrix<double, 4, 3> Q;
  getQMatrix(quat, Q);
  Eigen::Matrix<double, 4, 3> Qd;
  getQMatrix(quatd, Qd);
  angular_acceleration = 2.0 * Qd.transpose() * quatd + 2.0 * Q.transpose() * quatdd;
}

}

#endif /* QUATERNION_UTILITIES_H_ */
