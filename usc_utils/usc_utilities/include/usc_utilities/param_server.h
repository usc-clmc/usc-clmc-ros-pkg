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


#ifndef UTILITIES_PARAM_SERVER_H_
#define UTILITIES_PARAM_SERVER_H_

#include <roscpp_utilities/param_server.h>


namespace usc_utilities
{

  using roscpp_utilities::getParam;
  using roscpp_utilities::read;
  using roscpp_utilities::require;
  using roscpp_utilities::tokenizeString;
  using roscpp_utilities::readStringArraySpaceSeparated;
  using roscpp_utilities::appendLeadingSlash;
  using roscpp_utilities::appendTrailingSlash;
  using roscpp_utilities::removeLeadingSlash;
  using roscpp_utilities::getString;
  using roscpp_utilities::write;

  inline bool readIntArray(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<int>& array, const bool verbose = true)
  {
    return read(node_handle, parameter_name, array, verbose);
  }

  inline bool readDoubleArray(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<double>& array, const bool verbose = true)
  {
    return read(node_handle, parameter_name, array, verbose);
  }

  inline bool readEigenVector(ros::NodeHandle& node_handle, const std::string& parameter_name, Eigen::VectorXd& vector, const bool verbose = true)
  {
    return read(node_handle, parameter_name, vector, verbose);
  }

  inline bool getValue(XmlRpc::XmlRpcValue& config, double& value)
  {
    return getParam(config, value);
  }

} // namespace usc_utilities
#endif /* UTILITIES_PARAM_SERVER_H_ */
