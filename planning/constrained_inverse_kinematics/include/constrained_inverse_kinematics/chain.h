/*
 * chain.h
 *
 *  Created on: Jul 19, 2011
 *      Author: kalakris
 */

#ifndef CHAIN_H_
#define CHAIN_H_

#include <boost/shared_ptr.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <constrained_inverse_kinematics/fk_solver.h>

namespace constrained_inverse_kinematics
{

class Chain
{
public:
  Chain(const std::string& root, const std::string& tip);
  virtual ~Chain();

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  urdf::Model urdf_;
  boost::shared_ptr<const FKSolver> fk_solver_;
  int num_joints_;
  int num_links_;

  std::vector<boost::shared_ptr<const urdf::Joint> > joints_;

  void clipJointAnglesAtLimits(KDL::JntArray& jnt_array) const;

};

}

#endif /* CHAIN_H_ */
