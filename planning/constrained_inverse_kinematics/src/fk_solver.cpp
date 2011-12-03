/*
 * fk_solver.cpp
 *
 *  Created on: Jun 13, 2011
 *      Author: kalakris
 */

#include <constrained_inverse_kinematics/fk_solver.h>
#include <conversions/kdl_to_eigen.h>

using namespace KDL;
using namespace conversions;

namespace constrained_inverse_kinematics
{

FKSolver::FKSolver(const KDL::Chain& chain):
    chain_(chain)
{
  num_joints_ = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();

  for (unsigned int i=0; i<num_segments_; ++i)
  {
    if (chain_.getSegment(i).getJoint().getType() != Joint::None)
    {
      joint_to_segment_id_.push_back(i);
    }
  }
}

FKSolver::~FKSolver()
{
}

bool FKSolver::solve(const KDL::JntArray& q_in,
           KinematicsInfo& kinematics) const
{
  return solve(q_in, kinematics.joint_pos_, kinematics.joint_axis_, kinematics.link_frames_);
}

bool FKSolver::solve(const KDL::JntArray& q_in, std::vector<KDL::Vector>& joint_pos, std::vector<KDL::Vector>& joint_axis, std::vector<KDL::Frame>& segment_frames) const
{
  Frame p_out = Frame::Identity();

  if (q_in.rows() != num_joints_)
    return false;

  joint_pos.resize(num_joints_);
  joint_axis.resize(num_joints_);
  segment_frames.resize(num_segments_);

  int j = 0;
  for (unsigned int i = 0; i < num_segments_; i++)
  {
    double joint_value = 0.0;
    if (chain_.getSegment(i).getJoint().getType() != Joint::None)
    {
      joint_value = q_in(j);
      joint_pos[j] = p_out * chain_.getSegment(i).getJoint().JointOrigin();
      joint_axis[j] = p_out.M * chain_.getSegment(i).getJoint().JointAxis();
      j++;
    }
    p_out = p_out * chain_.getSegment(i).pose(joint_value);
    segment_frames[i] = p_out;
  }
  return true;
}

void FKSolver::getPositionJacobian(const KinematicsInfo& kinematics,
                         const KDL::Vector& position,
                         Eigen::MatrixXd& jacobian) const
{
  jacobian = Eigen::MatrixXd::Zero(3,num_joints_);
  Eigen::Vector3d joint_axis;
  Eigen::Vector3d joint_offset_e;
  for (unsigned int i=0; i<num_joints_; ++i)
  {
    const KDL::Joint& joint = chain_.getSegment(joint_to_segment_id_[i]).getJoint();
    const KDL::Joint::JointType& type = joint.getType();
    if (type == KDL::Joint::RotAxis ||
        type == KDL::Joint::RotX ||
        type == KDL::Joint::RotY ||
        type == KDL::Joint::RotZ)
    {
      KDL::Vector joint_offset = position - kinematics.joint_pos_[i];
      kdlVectorToEigenVector(kinematics.joint_axis_[i], joint_axis);
      kdlVectorToEigenVector(joint_offset, joint_offset_e);
      jacobian.col(i) = joint_axis.cross(joint_offset_e);
    }
    else if (type == KDL::Joint::TransAxis ||
             type == KDL::Joint::TransX ||
             type == KDL::Joint::TransY ||
             type == KDL::Joint::TransZ)
    {
      kdlVectorToEigenVector(kinematics.joint_axis_[i], joint_axis);
      jacobian.col(i) = joint_axis;
    }
  }
}

void FKSolver::getOrientationJacobian(const KinematicsInfo& kinematics,
                         Eigen::MatrixXd& jacobian) const
{
  jacobian = Eigen::MatrixXd::Zero(3,num_joints_);
  Eigen::Vector3d joint_axis;
  for (unsigned int i=0; i<num_joints_; ++i)
  {
    const KDL::Joint& joint = chain_.getSegment(joint_to_segment_id_[i]).getJoint();
    const KDL::Joint::JointType& type = joint.getType();
    if (type == KDL::Joint::RotAxis ||
        type == KDL::Joint::RotX ||
        type == KDL::Joint::RotY ||
        type == KDL::Joint::RotZ)
    {
      kdlVectorToEigenVector(kinematics.joint_axis_[i], joint_axis);
      jacobian.col(i) = joint_axis;
    }
  }
}

boost::shared_ptr<const FKSolver> FKSolver::clone() const
{
  boost::shared_ptr<const FKSolver> ret(new FKSolver(chain_));
  return ret;
}

}

