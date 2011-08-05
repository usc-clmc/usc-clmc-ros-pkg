/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		variable_name_map.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Feb 27, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>

#include <cassert>
#include <stdio.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/variable_name_map.h>

namespace pr2_dynamic_movement_primitive_controller
{

bool VariableNameMap::initialize(const std::vector<std::string>& supported_variable_names,
                                 const std::vector<std::string>& used_variable_names,
                                 const int start_index)
{
  supported_name_to_index_map_.clear();
  start_index_ = start_index;
  for (int i = start_index_; i < start_index_ + (int)supported_variable_names.size(); ++i)
  {
    supported_name_to_index_map_.insert(std::tr1::unordered_map<std::string, int>::value_type(supported_variable_names[i - start_index_], i));
  }

  used_to_supported_map_.resize(supported_variable_names.size(), -1);
  supported_to_used_map_.resize(supported_variable_names.size(), -1);

  for (int i = 0; i < (int)used_variable_names.size(); ++i)
  {
    int supported_index;
    if(!getSupportedVariableIndex(used_variable_names[i], supported_index))
    {
      ROS_ERROR("Could not match >%s<.", used_variable_names[i].c_str());
      return false;
    }
    used_to_supported_map_[i] = supported_index;
    supported_to_used_map_[supported_index - start_index_] = i;
  }

  return (initialized_ = true);
}

bool VariableNameMap::initialize(const std::vector<std::string>& supported_variable_names, const int start_index)
{
  std::vector<std::string> used_variable_names;
  return initialize(supported_variable_names, used_variable_names, start_index);
}

// REAL-TIME REQUIREMENTS
void VariableNameMap::reset()
{
  assert(initialized_);
  for (int i = 0; i < (int)used_to_supported_map_.size(); ++i)
  {
    used_to_supported_map_[i] = -1;
  }
}

// REAL-TIME REQUIREMENTS
bool VariableNameMap::set(const std::string& used_variable_name, const int index)
{
  assert(initialized_);
  if ((index < 0) || (index >= (int)used_to_supported_map_.size()))
  {
    return false;
  }
  std::tr1::unordered_map<std::string, int>::iterator it = supported_name_to_index_map_.find(used_variable_name);
  if(it == supported_name_to_index_map_.end())
  {
    return false;
  }
  int supported_index;
  if(!getSupportedVariableIndex(used_variable_name, supported_index))
  {
    return false;
  }
  used_to_supported_map_[index] = supported_index;
  supported_to_used_map_[supported_index - start_index_] = index;
  return true;
}

// REAL-TIME REQUIREMENTS
bool VariableNameMap::getSupportedVariableIndex(const int used_index, int& supported_index) const
{
  assert(initialized_);
  if((used_index >= 0) && (used_index < (int)used_to_supported_map_.size()))
  {
    if(used_to_supported_map_[used_index] >= 0)
    {
      supported_index = used_to_supported_map_[used_index];
      return true;
    }
  }
  return false;
}

// REAL-TIME REQUIREMENTS
bool VariableNameMap::getUsedVariableIndex(const int supported_index, int& used_index) const
{
  assert(initialized_);
  int index = supported_index - start_index_;
  if((index >= 0) && (index < (int)supported_to_used_map_.size()))
  {
    if(supported_to_used_map_[index] >= 0)
    {
      used_index = supported_to_used_map_[index];
      return true;
    }
  }
  return false;
}

// REAL-TIME REQUIREMENTS
bool VariableNameMap::getSupportedVariableIndex(const std::string& name, int& index) const
{
  std::tr1::unordered_map<std::string, int>::const_iterator it = supported_name_to_index_map_.find(name);
  if (it != supported_name_to_index_map_.end())
  {
    index = it->second;
    return true;
  }
  return false;
}

}
