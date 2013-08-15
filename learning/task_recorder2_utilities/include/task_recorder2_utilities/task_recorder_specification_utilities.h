/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_recorder_specification_utilities.h

  \author	Peter Pastor
  \date		Jun 19, 2011

 *********************************************************************/

#ifndef TASK_RECORDER_SPECIFICATION_UTILITIES_H_
#define TASK_RECORDER_SPECIFICATION_UTILITIES_H_

// system includes
#include <vector>
#include <ros/ros.h>

#include <boost/algorithm/string/predicate.hpp>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

// local includes
#include <task_recorder2_msgs/TaskRecorderSpecification.h>

namespace task_recorder2_utilities
{

inline bool readTaskRecorderSpecification(std::vector<task_recorder2_msgs::TaskRecorderSpecification>& specifications,
                                          ros::NodeHandle node_handle = ros::NodeHandle("/TaskRecorderManager"))
{
  specifications.clear();
  // read the list of description-label_type mapping from the param server
  XmlRpc::XmlRpcValue recorder_map;
  if (!node_handle.getParam("task_recorders", recorder_map))
  {
    ROS_ERROR("Couldn't find parameter >%s/task_recorders<", node_handle.getNamespace().c_str());
    return false;
  }
  if (recorder_map.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR(">%s/task_recorders must be a struct and not of type >%i<.",
        node_handle.getNamespace().c_str(), (int)recorder_map.getType());
    return false;
  }

  for (int i = 0; i < recorder_map.size(); ++i)
  {
    task_recorder2_msgs::TaskRecorderSpecification specification;

    if (!recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::CLASS_NAME))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", specification.CLASS_NAME.c_str());
      return false;
    }
    std::string class_name = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::CLASS_NAME];
    specification.class_name = class_name;

    if (!recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::TOPIC_NAME))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", task_recorder2_msgs::TaskRecorderSpecification::TOPIC_NAME.c_str());
      return false;
    }
    std::string topic_name = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::TOPIC_NAME];
    specification.topic_name = topic_name;

    std::string service_prefix = "";
    if (recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::SERVICE_PREFIX))
    {
      std::string aux = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::SERVICE_PREFIX];
      service_prefix = aux;
    }
    specification.service_prefix = service_prefix;

    std::string variable_name_prefix = "";
    if (recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::VARIABLE_NAME_PREFIX))
    {
      std::string aux = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::VARIABLE_NAME_PREFIX];
      variable_name_prefix = aux;
    }
    specification.variable_name_prefix = variable_name_prefix;

    double message_timer_rate = -1.0;
    if (recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::MESSAGE_TIMER_RATE))
    {
      double aux = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::MESSAGE_TIMER_RATE];
      message_timer_rate = aux;
    }
    specification.message_timer_rate = message_timer_rate;

    if (!recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD.c_str());
      return false;
    }
    std::string splining_method = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD];
    specification.splining_method = splining_method;

    specifications.push_back(specification);
    ROS_DEBUG("Class with name >%s< has topic named >%s<, service prefix >%s<, variable name prefix >%s<, and splining method >%s<.",
              class_name.c_str(), topic_name.c_str(), service_prefix.c_str(), variable_name_prefix.c_str(), splining_method.c_str());
  }
  return true;
}

inline bool getAllVariableNames(const std::vector<task_recorder2_msgs::TaskRecorderSpecification>& specifications,
                                std::vector<std::string>& variable_names)
{
  ros::NodeHandle node_handle = ros::NodeHandle("/TaskRecorderManager");
  variable_names.clear();
  for (int i = 0; i < (int)specifications.size(); ++i)
  {
    ROS_DEBUG("Getting all variable names for >%s<.", specifications[i].class_name.c_str());
    std::vector<std::string> names;
    ROS_VERIFY(usc_utilities::read(node_handle, specifications[i].class_name + "/variable_names", names));
    variable_names.insert(variable_names.end(), names.begin(), names.end());
  }
  return true;
}

}

#endif /* TASK_RECORDER_SPECIFICATION_UTILITIES_H_ */
