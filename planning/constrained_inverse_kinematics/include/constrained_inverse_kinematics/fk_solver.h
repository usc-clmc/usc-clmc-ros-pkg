/*
 * fk_solver.h
 *
 *  Created on: Jun 13, 2011
 *      Author: kalakris
 */

#ifndef FK_SOLVER_H_
#define FK_SOLVER_H_

#include <vector>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>

namespace constrained_inverse_kinematics
{

struct KinematicsInfo
{
  std::vector<KDL::Vector> joint_pos_;
  std::vector<KDL::Vector> joint_axis_;
  std::vector<KDL::Frame> link_frames_;
};

/**
 * This forward kinematics solver provides the positions and axes of all joints in the chain,
 * and all link frames.
 */
class FKSolver
{
public:
  FKSolver(const KDL::Chain& chain);
  virtual ~FKSolver();

  virtual bool solve(const KDL::JntArray& q_in,
             std::vector<KDL::Vector>& joint_pos,
             std::vector<KDL::Vector>& joint_axis,
             std::vector<KDL::Frame>& segment_frames) const;

  bool solve(const KDL::JntArray& q_in,
             KinematicsInfo& kinematics) const;

  /**
   * Returns a jacobian that is 3 x num_joints
   * @param kinematics
   * @param position
   * @param jacobian
   */
  void getPositionJacobian(const KinematicsInfo& kinematics,
                           const KDL::Vector& position,
                           Eigen::MatrixXd& jacobian) const;
  /**
   * Returns a jacobian that is 3 x num_joints
   * @param kinematics
   * @param jacobian
   */
  void getOrientationJacobian(const KinematicsInfo& kinematics,
                           Eigen::MatrixXd& jacobian) const;

  virtual boost::shared_ptr<const FKSolver> clone() const;

protected:
  KDL::Chain chain_;

private:
  unsigned int num_joints_;
  unsigned int num_segments_;
  std::vector<int> joint_to_segment_id_;

};

}

#endif /* FK_SOLVER_H_ */
