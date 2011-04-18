/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <policy_library/policy.h>

USING_PART_OF_NAMESPACE_EIGEN

namespace policy_library
{

bool Policy::computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices,
                                 const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                 const double weight, std::vector<Eigen::VectorXd>& control_costs)
{
    int num_dimensions = control_cost_matrices.size();
    int num_time_steps = parameters[0].size();

    std::vector<int> num_parameters(num_dimensions);
    this->getNumParameters(num_parameters);

    // initialize output vector if necessary:
    if (int(control_costs.size()) != num_dimensions)
    {
        control_costs.clear();
        for (int d=0; d<num_dimensions; ++d)
        {
            control_costs.push_back(VectorXd::Zero(num_time_steps));
        }
    }

    // compute the costs
    for (int d=0; d<num_dimensions; ++d)
    {
        for (int t=0; t<num_time_steps; ++t)
        {
            control_costs[d](t) = weight * parameters[d][t].dot(control_cost_matrices[d] * parameters[d][t]);
        }
    }

    return true;
}


}
