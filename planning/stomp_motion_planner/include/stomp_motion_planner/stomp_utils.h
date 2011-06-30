/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Mrinal Kalakrishnan */


#ifndef STOMP_UTILS_H_
#define STOMP_UTILS_H_

#include <kdl/jntarray.hpp>
#include <stomp_motion_planner/stomp_robot_model.h>
#include <iostream>
#include <Eigen/Core>

namespace stomp_motion_planner
{

static const int DIFF_RULE_LENGTH = 7;
static const int NUM_DIFF_RULES = 3;
// the differentiation rules (centered at the center)
static const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] = {
    {0, 0, -2/6.0, -3/6.0, 6/6.0, -1/6.0, 0},                   // velocity
    {0, -1/12.0, 16/12.0, -30/12.0, 16/12.0, -1/12.0, 0},       // acceleration
    {0, 1/12.0, -17/12.0, 46/12.0, -46/12.0, 17/12.0, -1/12.0}  // jerk
};

inline void debugJointArray(KDL::JntArray& joint_array)
{
  for (unsigned int i=0; i<joint_array.rows(); i++)
  {
    std::cout << joint_array(i) << "\t";
  }
  std::cout << std::endl;
}

template<typename KDLType, typename EigenType>
void kdlVecToEigenVec(std::vector<KDLType>& kdl_v, std::vector<Eigen::Map<EigenType> >& eigen_v, int rows, int cols)
{
  int size = kdl_v.size();
  eigen_v.clear();
  for (int i=0; i<size; i++)
  {
    eigen_v.push_back(Eigen::Map<EigenType>(kdl_v[i].data, rows, cols));
  }
}

template<typename KDLType, typename EigenType>
void kdlVecVecToEigenVecVec(std::vector<std::vector<KDLType> >& kdl_vv, std::vector<std::vector<Eigen::Map<EigenType> > > & eigen_vv, int rows, int cols)
{
  int size = kdl_vv.size();
  eigen_vv.resize(size);
  for (int i=0; i<size; i++)
  {
    kdlVecToEigenVec(kdl_vv[i], eigen_vv[i], rows, cols);
  }
}


} //namespace stomp

#endif /* STOMP_UTILS_H_ */
