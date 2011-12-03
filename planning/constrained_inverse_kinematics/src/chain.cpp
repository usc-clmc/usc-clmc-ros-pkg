/*
 * chain.cpp
 *
 *  Created on: Jul 19, 2011
 *      Author: kalakris
 */

#include <constrained_inverse_kinematics/chain.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <kdl_parser/kdl_parser.hpp>

namespace constrained_inverse_kinematics
{

Chain::Chain(const std::string& root, const std::string& tip)
{
  // load urdf:
  ROS_VERIFY(urdf_.initParam("/robot_description"));

  // create kdl tree:
  ROS_VERIFY(kdl_parser::treeFromUrdfModel(urdf_, kdl_tree_));

  // get kdl chain:
  ROS_VERIFY(kdl_tree_.getChain(root, tip, kdl_chain_));

  fk_solver_.reset(new FKSolver(kdl_chain_));

  num_joints_ = kdl_chain_.getNrOfJoints();
  num_links_ = kdl_chain_.getNrOfSegments();

  for (int i=0; i<num_links_; ++i)
  {
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      joints_.push_back(urdf_.getJoint(kdl_chain_.getSegment(i).getJoint().getName()));
    }
  }

  ROS_ASSERT(joints_.size() == (unsigned int)num_joints_);

}

void Chain::clipJointAnglesAtLimits(KDL::JntArray& jnt_array) const
{
  for (unsigned int i=0; i<jnt_array.rows(); ++i)
  {
    if (jnt_array(i) > joints_[i]->limits->upper)
      jnt_array(i) = joints_[i]->limits->upper;
    if (jnt_array(i) < joints_[i]->limits->lower)
      jnt_array(i) = joints_[i]->limits->lower;
  }
}

Chain::~Chain()
{
}

}
