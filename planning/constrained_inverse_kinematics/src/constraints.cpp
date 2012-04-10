/*
 * constraints.cpp
 *
 *  Created on: Jun 15, 2011
 *      Author: kalakris
 */

#include <constrained_inverse_kinematics/constraints.h>
#include <conversions/ros_to_kdl.h>
#include <conversions/kdl_to_eigen.h>

using namespace conversions;

namespace constrained_inverse_kinematics
{

void Constraints::clear()
{
  primary_constraints_.clear();
}

void Constraints::addPrimaryConstraint(boost::shared_ptr<Constraint> constraint)
{
  primary_constraints_.push_back(constraint);
}

bool OrientationConstraint::initialize(const arm_navigation_msgs::OrientationConstraint& constraint,
                const KDL::Chain& chain)
{
  constraint_ = constraint;
  link_id_ = -1;
  for (unsigned int i=0; i<chain.getNrOfSegments(); ++i)
  {
    if (constraint_.link_name == chain.getSegment(i).getName())
    {
      link_id_ = i;
      break;
    }
  }
  if (link_id_ == -1)
  {
    ROS_ERROR("OrientationConstraint: couldn't find link %s in chain.", constraint_.link_name.c_str());
    return false;
  }
  return true;
}

bool OrientationConstraint::getErrorAndJacobian(const KinematicsInfo& kinematics,
                                 Eigen::VectorXd& error,
                                 Eigen::MatrixXd& jacobian)
{
  return true;
}

OrientationConstraint::OrientationConstraint(boost::shared_ptr<const FKSolver>& fk_solver):
  fk_solver_(fk_solver)
{
}

PositionConstraint::PositionConstraint(boost::shared_ptr<const FKSolver>& fk_solver):
  fk_solver_(fk_solver)
{
}

bool PositionConstraint::initialize(const arm_navigation_msgs::PositionConstraint& constraint,
                const KDL::Chain& chain)
{
  constraint_ = constraint;
  link_id_ = -1;
  for (unsigned int i=0; i<chain.getNrOfSegments(); ++i)
  {
    if (constraint_.link_name == chain.getSegment(i).getName())
    {
      link_id_ = i;
      break;
    }
  }
  if (link_id_ == -1)
  {
    ROS_ERROR("CIK PositionConstraint: couldn't find link %s in chain.", constraint_.link_name.c_str());
    return false;
  }

  rosPointToKdlVector(constraint_.target_point_offset, offset_);
  rosPointToKdlVector(constraint_.position, desired_location_);

  if (constraint_.constraint_region_shape.type != arm_navigation_msgs::Shape::BOX ||
      constraint_.constraint_region_shape.dimensions.size() != 3)
  {
    ROS_ERROR("CIK PositionConstraint: we only handle 3-d BOX constraints currently.");
    return false;
  }

  KDL::Rotation constraint_rotation;
  rosQuaternionToKdlRotation(constraint_.constraint_region_orientation, constraint_rotation);
  constraint_rotation_inverse_ = constraint_rotation.Inverse();
  kdlRotationToEigenMatrix3d(constraint_rotation_inverse_, constraint_rotation_inverse_eigen_);

  return true;
}

bool PositionConstraint::getErrorAndJacobian(const KinematicsInfo& kinematics,
                                 Eigen::VectorXd& error,
                                 Eigen::MatrixXd& jacobian)
{
  KDL::Vector cur_position = kinematics.link_frames_[link_id_] * offset_;

  // compute the error in world frame
  KDL::Vector error_world = desired_location_ - cur_position;

  // rotate the error into the constraint frame, so that we can split up each component
  KDL::Vector error_constraint = constraint_rotation_inverse_ * error_world;

  // check each component (x,y,z) to see if it's within bounds
  int num_output_rows = 0;
  bool components_within_bounds[3];
  bool all_within_bounds = true;
  for (int i=0; i<3; ++i)
  {
    components_within_bounds[i] =
        (fabs(error_constraint(i)) < constraint_.constraint_region_shape.dimensions[i]);
    if (!components_within_bounds[i])
    {
      all_within_bounds = false;
      ++num_output_rows;
    }
  }
  if (all_within_bounds)
    return false;

  // get the jacobian
  Eigen::MatrixXd world_jacobian;
  Eigen::MatrixXd constraint_jacobian;
  fk_solver_->getPositionJacobian(kinematics, cur_position, world_jacobian);

  // transform the jacobian into constraint space
  constraint_jacobian = constraint_rotation_inverse_eigen_ * world_jacobian;

  // initialize outputs
  int num_joints = constraint_jacobian.cols();
  error = Eigen::VectorXd::Zero(num_output_rows);
  jacobian = Eigen::VectorXd::Zero(num_output_rows, num_joints);

  int index=0;
  for (int i=0; i<3; ++i)
  {
    if (!components_within_bounds[i])
    {
      error(index) = error_constraint(i);
      jacobian.row(index) = constraint_jacobian.row(i);
      ++index;
    }
  }

  return true;
}

}
