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
#include <iostream>
#include <Eigen/Core>

namespace stomp
{

static const int DIFF_RULE_LENGTH = 7;
static const int TRAJECTORY_PADDING = DIFF_RULE_LENGTH - 1;
static const int NUM_DIFF_RULES = 4;
// the differentiation rules (centered at the center)
static const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] = {
    {0,       0,        0,        1,        0,       0,       0},       // position
    {0,       0,   -2/6.0,   -3/6.0,    6/6.0,  -1/6.0,       0},       // velocity
    {0, -1/12.0,  16/12.0, -30/12.0,  16/12.0, -1/12.0,       0},       // acceleration
    {0,  1/12.0, -17/12.0,  46/12.0, -46/12.0, 17/12.0, -1/12.0}        // jerk
};

enum CostComponents
{
  STOMP_POSITION = 0,
  STOMP_VELOCITY = 1,
  STOMP_ACCELERATION = 2,
  STOMP_JERK = 3
};

static inline void differentiate(const Eigen::VectorXd& input,
                                 CostComponents order, Eigen::VectorXd& output,
                                 double dt)
{
  double multiplier = 1.0/pow(dt,(int)order);
  int T = input.rows();
  output = Eigen::VectorXd::Zero(T);
  for (int i=0; i<T; ++i)
  {
    output(i) = 0.0;
    for (int j=-DIFF_RULE_LENGTH/2; j<=DIFF_RULE_LENGTH/2; ++j)
    {
      int index = i+j;
      if (index < 0 || index >= T)
        continue;
      output(i) += multiplier * DIFF_RULES[order][j+DIFF_RULE_LENGTH/2] * input(index);
    }
  }
}

} //namespace stomp

#endif /* STOMP_UTILS_H_ */
