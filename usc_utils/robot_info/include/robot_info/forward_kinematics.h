/*
 * forward_kinematics.h
 *
 *  Created on: Dec 10, 2010
 *      Author: kalakris
 */

#ifndef FORWARD_KINEMATICS_H_
#define FORWARD_KINEMATICS_H_

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Pose.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <usc_utilities/kdl_chain_wrapper.h>

namespace robot_info
{

/**
 * Class for forward kinematics computations.
 *
 * \warning This class is not thread-safe, please create one copy per thread!
 */
class ForwardKinematics
{
public:

  ForwardKinematics()
    : initialized_(false) {};
  virtual ~ForwardKinematics() {};

  /*! Initializes the kinematic chain from the urdf provided the start and end link
   * @param start_link
   * @param end_link
   * @return True on success, otherwise False
   */
  bool initialize(const std::string& start_link, const std::string& end_link);

  /*!
   * @param joint_positions
   * @param pose
   */
  void forwardKinematics(const std::vector<double>& joint_positions, geometry_msgs::Pose& pose);

private:
  bool initialized_;

  usc_utilities::KDLChainWrapper chain_;
  KDL::JntArray kdl_joint_array_;

};

}

#endif /* FORWARD_KINEMATICS_H_ */
