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

#include <string>
#include <vector>
#include <sstream>

#include <ros/node_handle.h>
#include <ros/assert.h>

#include <geometry_msgs/Pose.h>

#include <Eigen/Core>

namespace usc_utilities
{
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, int& value);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& value);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::string& str);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<int>& int_array);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<double>& double_array);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<std::string>& str_array);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, geometry_msgs::Point& position);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, geometry_msgs::Quaternion& position);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, geometry_msgs::Pose& pose);
bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value);
bool getValue(XmlRpc::XmlRpcValue& config, double& value);

bool readDoubleArray(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<double>& array);
bool readEigenVector(ros::NodeHandle& node_handle, const std::string& parameter_name, Eigen::VectorXd& vector);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, std::string& parameter_value);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, double& parameter_value);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, int& parameter_value);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, bool& parameter_value);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<double>& array);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& str_array);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, geometry_msgs::Point& position);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, geometry_msgs::Quaternion& position);
bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, geometry_msgs::Pose& pose);
void tokenizeString(const std::string& str_array, std::vector<std::string>& array);
bool readStringArraySpaceSeparated(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& array);
void appendTrailingSlash(std::string& directory_name);
void removeLeadingSlash(std::string& topic_name);
std::string getString(const int number);

//// inline functions follow:

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, int& value)
{
  if (!config.hasMember(key))
  {
    return false;
  }
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    return false;
  }
  value = param;
  return true;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& value)
{
  if (!config.hasMember(key))
  {
    return false;
  }
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    return false;
  }
  value = param;
  return true;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::string& str)
{
  if (!config.hasMember(key))
  {
    return false;
  }

  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    return false;
  }
  str = std::string(param);
  return true;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<int>& int_array)
{
  if (!config.hasMember(key))
  {
    return false;
  }

  XmlRpc::XmlRpcValue i_array_xml = config[key];

  if (i_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return false;
  }

  int_array.clear();
  for (int i=0; i<i_array_xml.size(); ++i)
  {
    if (i_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      return false;
    }
    int_array.push_back(static_cast<int>(i_array_xml[i]));
  }
  return true;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<double>& double_array)
{
  if (!config.hasMember(key))
  {
    ROS_ERROR("XmlRpcValue does not contain key %s.", key.c_str());
    return false;
  }

  XmlRpc::XmlRpcValue d_array_xml = config[key];

  if (d_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("XmlRpcValue is not of type array.");
    return false;
  }

  double_array.clear();
  for (int i=0; i<d_array_xml.size(); ++i)
  {
    if (d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
      return false;
    }
    double value = 0.0;
    if (d_array_xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = static_cast<double>(static_cast<int>(d_array_xml[i]));
    }
    else
    {
      value = static_cast<double>(d_array_xml[i]);
    }
    double_array.push_back(value);
  }

  return true;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<std::string>& str_array)
{
  if (!config.hasMember(key))
  {
    ROS_ERROR("XmlRpcValue does not contain key %s.", key.c_str());
    return false;
  }

  XmlRpc::XmlRpcValue str_array_xml = config[key];
  if (str_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("XmlRpcValue is not of type array.");
    return false;
  }

  str_array.clear();
  for (int i=0; i<str_array_xml.size(); ++i)
  {
    str_array.push_back(std::string(str_array_xml[i]));
  }
  return true;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value)
{
  if (!config.hasMember(key))
    {
      return false;
    }
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
  {
    return false;
  }
  value = param;
  return true;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, geometry_msgs::Point& position)
{
  if (!config.hasMember(key))
  {
    ROS_ERROR("XmlRpcValue does not contain >%s<.", key.c_str());
    return false;
  }
  XmlRpc::XmlRpcValue xml_position = config[key];
  if ( (!xml_position.hasMember("x")) || (!xml_position.hasMember("y")) || (!xml_position.hasMember("z")))
  {
    ROS_ERROR("XmlRpcValue does not contain >%s, %s, %s<.", std::string(key+"/x").c_str(), std::string(key+"/y").c_str(), std::string(key+"/z").c_str());
    return false;
  }
  XmlRpc::XmlRpcValue x_param = xml_position["x"];
  if (x_param.getType() != XmlRpc::XmlRpcValue::TypeDouble && x_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
    return false;
  }
  XmlRpc::XmlRpcValue y_param = xml_position["y"];
  if (y_param.getType() != XmlRpc::XmlRpcValue::TypeDouble && y_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
    return false;
  }
  XmlRpc::XmlRpcValue z_param = xml_position["z"];
  if (z_param.getType() != XmlRpc::XmlRpcValue::TypeDouble && z_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
    return false;
  }
  bool ret = true;
  ret = ret && getValue(x_param, position.x);
  ret = ret && getValue(y_param, position.y);
  ret = ret && getValue(z_param, position.z);
  return ret;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, geometry_msgs::Quaternion& orientation)
{
  if (!config.hasMember(key))
  {
    ROS_ERROR("XmlRpcValue does not contain >%s<.", key.c_str());
    return false;
  }
  XmlRpc::XmlRpcValue xml_orientation = config[key];
  if ( (!xml_orientation.hasMember("w")) || (!xml_orientation.hasMember("x")) || (!xml_orientation.hasMember("y")) || (!xml_orientation.hasMember("z")))
  {
    ROS_ERROR("XmlRpcValue does not contain >w, x, y, z<.");
    return false;
  }
  XmlRpc::XmlRpcValue w_param = xml_orientation["w"];
  if (w_param.getType() != XmlRpc::XmlRpcValue::TypeDouble && w_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
    return false;
  }
  XmlRpc::XmlRpcValue x_param = xml_orientation["x"];
  if (x_param.getType() != XmlRpc::XmlRpcValue::TypeDouble && x_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
    return false;
  }
  XmlRpc::XmlRpcValue y_param = xml_orientation["y"];
  if (y_param.getType() != XmlRpc::XmlRpcValue::TypeDouble && y_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
    return false;
  }
  XmlRpc::XmlRpcValue z_param = xml_orientation["z"];
  if (z_param.getType() != XmlRpc::XmlRpcValue::TypeDouble && z_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
    return false;
  }

  bool ret = true;
  ret = ret && getValue(w_param, orientation.w);
  ret = ret && getValue(x_param, orientation.x);
  ret = ret && getValue(y_param, orientation.y);
  ret = ret && getValue(z_param, orientation.z);
  return ret;
}

inline bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, geometry_msgs::Pose& pose)
{
  if(!config.hasMember(key))
  {
    ROS_ERROR("XmlRpcValue does not contain >%s<.", key.c_str());
    return false;
  }
  XmlRpc::XmlRpcValue xml_pose = config[key];
  bool ret = true;
  ret = ret && getParam(xml_pose, "position", pose.position);
  ret = ret && getParam(xml_pose, "orientation", pose.orientation);
  return ret;
}

inline bool getValue(XmlRpc::XmlRpcValue& config, double& value)
{
  if (config.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    value = static_cast<double>(static_cast<int>(config));
  }
  else if(config.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    value = static_cast<double>(config);
  }
  else
  {
    ROS_ERROR("XmlRpcValue is either an integer nor a double. Cannot get value.");
    return false;
  }
  return true;
}

inline bool readDoubleArray(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<double>& array)
{
  XmlRpc::XmlRpcValue d_array_xml;
  if(!node_handle.getParam(parameter_name, d_array_xml))
  {
    ROS_ERROR("Could not retrieve parameter %s in namespace %s.", parameter_name.c_str(), node_handle.getNamespace().c_str());
    return false;
  }

  if (d_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("XmlRpcValue is not of type array.");
    return false;
  }

  array.clear();
  for (int i=0; i<d_array_xml.size(); ++i)
  {
    if (d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
      return false;
    }
    double value = 0.0;
    if (d_array_xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = static_cast<double>(static_cast<int>(d_array_xml[i]));
    }
    else
    {
      value = static_cast<double>(d_array_xml[i]);
    }
    array.push_back(value);
  }

  return true;
}

inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<double>& array)
{
  return readDoubleArray(node_handle, parameter_name, array);
}

inline bool readEigenVector(ros::NodeHandle& node_handle, const std::string& parameter_name, Eigen::VectorXd& vector)
{
  std::vector<double> array;
  if (!readDoubleArray(node_handle, parameter_name, array))
  {
    return false;
  }

  vector = Eigen::VectorXd::Zero(array.size());
  for (int i=0; i<static_cast<int>(array.size()); ++i)
  {
    vector(i) = array[i];
  }

  return true;
}

inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& str_array)
{
  XmlRpc::XmlRpcValue list;
  if(!node_handle.getParam(parameter_name, list))
  {
    ROS_ERROR("Could not retrieve parameter %s in namespace %s.", parameter_name.c_str(), node_handle.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue str_array_xml = list;
  if (str_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("XmlRpcValue is not of type array.");
    return false;
  }

  str_array.clear();
  for (int i=0; i<str_array_xml.size(); ++i)
  {
    str_array.push_back(std::string(str_array_xml[i]));
  }
  return true;

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

inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, geometry_msgs::Point& position)
{
  bool ret = true;
  ret = ret && read(node_handle, parameter_name+"/x", position.x);
  ret = ret && read(node_handle, parameter_name+"/y", position.y);
  ret = ret && read(node_handle, parameter_name+"/z", position.z);
  return ret;
}

inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, geometry_msgs::Quaternion& orientation)
{
  bool ret = true;
  ret = ret && read(node_handle, parameter_name+"/x", orientation.x);
  ret = ret && read(node_handle, parameter_name+"/y", orientation.y);
  ret = ret && read(node_handle, parameter_name+"/z", orientation.z);
  ret = ret && read(node_handle, parameter_name+"/w", orientation.w);
  return ret;
}

inline bool read(ros::NodeHandle& node_handle, const std::string& parameter_name, geometry_msgs::Pose& pose)
{
  bool ret = true;
  ret = ret && read(node_handle, parameter_name+"/position", pose.position);
  ret = ret && read(node_handle, parameter_name+"/orientation", pose.orientation);
  return ret;
}

inline void tokenizeString(const std::string& str_array, std::vector<std::string>& array)
{
  array.clear();
  std::stringstream str_stream(str_array);
  std::string item;
  while (str_stream >> item)
  {
    array.push_back(item);
  }
}

inline bool readStringArraySpaceSeparated(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& array)
{
  std::string str_array;

  if (!node_handle.getParam(parameter_name, str_array))
  {
    ROS_ERROR("Parameter %s/%s not found!", node_handle.getNamespace().c_str(), parameter_name.c_str());
    return false;
  }

  tokenizeString(str_array, array);
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

} // namespace usc_utilities
#endif /* UTILITIES_PARAM_SERVER_H_ */
