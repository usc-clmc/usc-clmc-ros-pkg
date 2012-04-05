/*
 * constraints.h
 *
 *  Created on: Jun 15, 2011
 *      Author: kalakris
 */

#ifndef CONSTRAINTS_H_
#define CONSTRAINTS_H_

#include <arm_navigation_msgs/Constraints.h>
#include <constrained_inverse_kinematics/fk_solver.h>

namespace constrained_inverse_kinematics
{

class ConstrainedIKSolver;

class Constraint
{
public:
  /**
   * Returns the error vector and a jacobian matrix for this error vector.
   * Returns false if the jacobian
   *
   * @param kinematics
   * @param error
   * @param jacobian
   * @return
   */
  virtual bool getErrorAndJacobian(const KinematicsInfo& kinematics,
                                   Eigen::VectorXd& error,
                                   Eigen::MatrixXd& jacobian)=0;
  friend class ConstrainedIKSolver;
};

class PositionConstraint: public Constraint
{
public:
  PositionConstraint(boost::shared_ptr<const FKSolver>& fk_solver);
  bool initialize(const arm_navigation_msgs::PositionConstraint& constraint,
                  const KDL::Chain& chain);
  virtual bool getErrorAndJacobian(const KinematicsInfo& kinematics,
                                   Eigen::VectorXd& error,
                                   Eigen::MatrixXd& jacobian);

  friend class ConstrainedIKSolver;

private:
  boost::shared_ptr<const FKSolver> fk_solver_;
  arm_navigation_msgs::PositionConstraint constraint_;
  int link_id_;
  KDL::Vector offset_;
  KDL::Vector desired_location_;
  KDL::Rotation constraint_rotation_inverse_;
  Eigen::Matrix3d constraint_rotation_inverse_eigen_;

};

class OrientationConstraint: public Constraint
{
public:
  OrientationConstraint(boost::shared_ptr<const FKSolver>& fk_solver);
  bool initialize(const arm_navigation_msgs::OrientationConstraint& constraint,
                  const KDL::Chain& chain);
  virtual bool getErrorAndJacobian(const KinematicsInfo& kinematics,
                                   Eigen::VectorXd& error,
                                   Eigen::MatrixXd& jacobian);
  friend class ConstrainedIKSolver;
private:
  boost::shared_ptr<const FKSolver> fk_solver_;
  arm_navigation_msgs::OrientationConstraint constraint_;
  int link_id_;
};

class Constraints
{
public:
  void clear();
  void addPrimaryConstraint(boost::shared_ptr<Constraint> constraint);
  friend class ConstrainedIKSolver;
private:
  std::vector<boost::shared_ptr<Constraint> > primary_constraints_;

};

}

#endif /* CONSTRAINTS_H_ */
