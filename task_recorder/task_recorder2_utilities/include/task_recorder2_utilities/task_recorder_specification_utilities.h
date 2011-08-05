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

#include <usc_utilities/param_server.h>
#include <task_recorder2_msgs/TaskRecorderSpecification.h>

// local includes

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

    if (!recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD.c_str());
      return false;
    }
    std::string splining_method = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD];
    specification.splining_method = splining_method;

    specifications.push_back(specification);
    ROS_DEBUG("Class with name >%s< has topic named >%s< and splining method >%s<.", class_name.c_str(), topic_name.c_str(), splining_method.c_str());
  }
  return true;
}

inline bool getAllVariableNames(const std::vector<task_recorder2_msgs::TaskRecorderSpecification>& specifications, std::vector<std::string>& variable_names)
{
  variable_names.clear();
  for (int i = 0; i < (int)specifications.size(); ++i)
  {
    if(specifications[i].class_name.compare("JointStatesRecorder") == 0)
    {
      variable_names.push_back("R_SFE_th");
      variable_names.push_back("R_SAA_th");
      variable_names.push_back("R_HR_th");
      variable_names.push_back("R_EB_th");
      variable_names.push_back("R_WR_th");
      variable_names.push_back("R_WFE_th");
      variable_names.push_back("R_WAA_th");
      variable_names.push_back("R_SFE_thd");
      variable_names.push_back("R_SAA_thd");
      variable_names.push_back("R_HR_thd");
      variable_names.push_back("R_EB_thd");
      variable_names.push_back("R_WR_thd");
      variable_names.push_back("R_WFE_thd");
      variable_names.push_back("R_WAA_thd");
      variable_names.push_back("R_SFE_u");
      variable_names.push_back("R_SAA_u");
      variable_names.push_back("R_HR_u");
      variable_names.push_back("R_EB_u");
      variable_names.push_back("R_WR_u");
      variable_names.push_back("R_WFE_u");
      variable_names.push_back("R_WAA_u");
    }
    if(specifications[i].class_name.compare("WrenchStatesRecorder") == 0)
    {
      variable_names.push_back("palm_force_x");
      variable_names.push_back("palm_force_y");
      variable_names.push_back("palm_force_z");
      variable_names.push_back("palm_torque_x");
      variable_names.push_back("palm_torque_y");
      variable_names.push_back("palm_torque_z");
    }
    if(specifications[i].class_name.compare("StrainGaugeStatesRecorder") == 0)
    {
      variable_names.push_back("R_RF");
      variable_names.push_back("R_MF");
      variable_names.push_back("R_LF");
    }
    if(specifications[i].class_name.compare("PressureSensorStatesRecorder") == 0)
    {
      std::vector<std::string> pressure_sensor_pad_names;
      pressure_sensor_pad_names.push_back("palm_");
      pressure_sensor_pad_names.push_back("right_finger_");
      pressure_sensor_pad_names.push_back("middle_finger_");
      pressure_sensor_pad_names.push_back("left_finger_");
      const unsigned int NUM_PRESSURE_SENSOR_PADS = 4;
      const unsigned int NUM_PRESSURE_SENSORS_PER_PAD = 24;
      for (int i = 0; i < (int)NUM_PRESSURE_SENSOR_PADS; ++i)
      {
        for (int j = 0; j < (int)NUM_PRESSURE_SENSORS_PER_PAD; ++j)
        {
          std::stringstream ss;
          ss << j;
          variable_names.push_back(pressure_sensor_pad_names[i] + ss.str());
        }
      }
    }
    if(specifications[i].class_name.compare("AudioRecorder") == 0)
    {
      const int NUM_AUDIO_SIGNALS = 28;
      for (int i = 0; i < NUM_AUDIO_SIGNALS; ++i)
      {
        variable_names.push_back(std::string("audio_") + usc_utilities::getString(i));
      }
    }
  }
  return true;
}


}


#endif /* TASK_RECORDER_SPECIFICATION_UTILITIES_H_ */
