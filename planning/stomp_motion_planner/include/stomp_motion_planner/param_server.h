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


#ifndef POLICY_IMPROVEMENT_UTILITIES_PARAM_SERVER_H_
#define POLICY_IMPROVEMENT_UTILITIES_PARAM_SERVER_H_

#include <string>
#include <vector>
#include <sstream>

#include <ros/node_handle.h>
#include <ros/assert.h>

#include <Eigen/Core>

namespace stomp_motion_planner
{

inline bool readDoubleArray(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<double>& array)
{
    XmlRpc::XmlRpcValue list;
    node_handle.getParam(parameter_name, list);

    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Parameter %s/%s needs to be an *array* of doubles", node_handle.getNamespace().c_str(), parameter_name.c_str());
        return false;
    }

    array.clear();
    for (int32_t i = 0; i < list.size(); ++i)
    {
        if (list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            ROS_ERROR("Parameter %s/%s needs to be an array of *doubles*", node_handle.getNamespace().c_str(), parameter_name.c_str());
            return false;
        }
        double value=0.0;
        if (list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
            value = static_cast<double>(static_cast<int>(list[i]));
        else value = static_cast<double>(list[i]);
        array.push_back(value);
    }
    return true;
}

inline bool readEigenVector(ros::NodeHandle& node_handle, const std::string& parameter_name, Eigen::VectorXd& vector)
{
    std::vector<double> array;
    if (!readDoubleArray(node_handle, parameter_name, array))
    {
        return false;
    }

    vector = Eigen::VectorXd::Zero(array.size());
    for (int i=0; i<int(array.size()); ++i)
    {
        vector(i) = array[i];
    }

    return true;
}

// TODO: make this a templated function
inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, std::string& parameter_value)
{
    if (!node_handle.getParam(parameter_name, parameter_value))
    {
        ROS_ERROR("Parameter %s/%s not found!", node_handle.getNamespace().c_str(), parameter_name.c_str());
        return false;
    }
    return true;
}
inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, double& parameter_value)
{
    if (!node_handle.getParam(parameter_name, parameter_value))
    {
        ROS_ERROR("Parameter %s/%s not found!", node_handle.getNamespace().c_str(), parameter_name.c_str());
        return false;
    }
    return true;
}
inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, int& parameter_value)
{
    if (!node_handle.getParam(parameter_name, parameter_value))
    {
        ROS_ERROR("Parameter %s/%s not found!", node_handle.getNamespace().c_str(), parameter_name.c_str());
        return false;
    }
    return true;
}
inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, bool& parameter_value)
{
    if (!node_handle.getParam(parameter_name, parameter_value))
    {
        ROS_ERROR("Parameter %s/%s not found!", node_handle.getNamespace().c_str(), parameter_name.c_str());
        return false;
    }
    return true;
}

inline bool readStringArraySpaceSeparated(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& array)
{
    std::string str_array;

    if (!node_handle.getParam(parameter_name, str_array))
    {
        ROS_ERROR("Parameter %s/%s not found!", node_handle.getNamespace().c_str(), parameter_name.c_str());
        return false;
    }

    array.clear();
    std::stringstream str_stream(str_array);
    std::string item;
    while (str_stream >> item)
    {
        array.push_back(item);
    }
    return true;
}

inline void appendTrailingSlash(std::string& directory_name)
{
    if (directory_name.compare(directory_name.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        directory_name.append("/");
    }
}
inline void removeLeadingSlash(std::string& topic_name)
{
    if (topic_name.compare(0, 1, "/") == 0) // the topic name starts with a slash
    {
        topic_name.assign(topic_name.substr(1, topic_name.length()));
    }
}

inline std::string getString(const int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

} // namespace policy_improvement_utilities
#endif /* POLICY_IMPROVEMENT_UTILITIES_PARAM_SERVER_H_ */
