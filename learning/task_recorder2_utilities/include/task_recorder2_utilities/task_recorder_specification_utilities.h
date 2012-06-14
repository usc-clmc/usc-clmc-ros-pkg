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

    std::string service_prefix = "";
    if (recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::SERVICE_PREFIX))
    {
      std::string aux = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::SERVICE_PREFIX];
      service_prefix = aux;
      specification.service_prefix = service_prefix;
    }

    if (!recorder_map[i].hasMember(task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD.c_str());
      return false;
    }
    std::string splining_method = recorder_map[i][task_recorder2_msgs::TaskRecorderSpecification::SPLINING_METHOD];
    specification.splining_method = splining_method;

    specifications.push_back(specification);
    ROS_DEBUG("Class with name >%s< has topic named >%s<, service prefix >%s< and splining method >%s<.",
              class_name.c_str(), topic_name.c_str(), service_prefix.c_str(), splining_method.c_str());
  }
  return true;
}

inline bool getAllVariableNames(const std::vector<task_recorder2_msgs::TaskRecorderSpecification>& specifications,
                                std::vector<std::string>& variable_names)
{

  // TODO: re-think this... after the next test

  const std::string RIGHT_ARM = "RightArm";
  const std::string LEFT_ARM = "LeftArm";
  const std::string RIGHT_ARM_PREFIX = "R_";
  const std::string LEFT_ARM_PREFIX = "L_";

  variable_names.clear();
  for (int i = 0; i < (int)specifications.size(); ++i)
  {
    ROS_DEBUG("Getting all variable names for >%s<.", specifications[i].class_name.c_str());

    if(specifications[i].class_name.compare("JointStatesRecorder") == 0)
    {
			// head
      variable_names.push_back("LPAN_th");
      variable_names.push_back("LTILT_th");
      variable_names.push_back("UPAN_th");
      variable_names.push_back("UTILT_th");
      variable_names.push_back("LPAN_thd");
      variable_names.push_back("LTILT_thd");
      variable_names.push_back("UPAN_thd");
      variable_names.push_back("UTILT_thd");
      variable_names.push_back("LPAN_u");
      variable_names.push_back("LTILT_u");
      variable_names.push_back("UPAN_u");
      variable_names.push_back("UTILT_u");

			// right arm
      variable_names.push_back("R_SFE_th");
      variable_names.push_back("R_SAA_th");
      variable_names.push_back("R_HR_th");
      variable_names.push_back("R_EB_th");
      variable_names.push_back("R_WR_th");
      variable_names.push_back("R_WFE_th");
      variable_names.push_back("R_WAA_th");
      variable_names.push_back("R_FR_th");
      variable_names.push_back("R_RF_th");
      variable_names.push_back("R_MF_th");
      variable_names.push_back("R_LF_th");
      variable_names.push_back("R_SFE_thd");
      variable_names.push_back("R_SAA_thd");
      variable_names.push_back("R_HR_thd");
      variable_names.push_back("R_EB_thd");
      variable_names.push_back("R_WR_thd");
      variable_names.push_back("R_WFE_thd");
      variable_names.push_back("R_WAA_thd");
      variable_names.push_back("R_FR_thd");
      variable_names.push_back("R_RF_thd");
      variable_names.push_back("R_MF_thd");
      variable_names.push_back("R_LF_thd");
      variable_names.push_back("R_SFE_u");
      variable_names.push_back("R_SAA_u");
      variable_names.push_back("R_HR_u");
      variable_names.push_back("R_EB_u");
      variable_names.push_back("R_WR_u");
      variable_names.push_back("R_WFE_u");
      variable_names.push_back("R_WAA_u");
      variable_names.push_back("R_FR_u");
      variable_names.push_back("R_RF_u");
      variable_names.push_back("R_MF_u");
      variable_names.push_back("R_LF_u");

			// left arm
      variable_names.push_back("L_SFE_th");
      variable_names.push_back("L_SAA_th");
      variable_names.push_back("L_HR_th");
      variable_names.push_back("L_EB_th");
      variable_names.push_back("L_WR_th");
      variable_names.push_back("L_WFE_th");
      variable_names.push_back("L_WAA_th");
      variable_names.push_back("L_FR_th");
      variable_names.push_back("L_RF_th");
      variable_names.push_back("L_MF_th");
      variable_names.push_back("L_LF_th");
      variable_names.push_back("L_SFE_thd");
      variable_names.push_back("L_SAA_thd");
      variable_names.push_back("L_HR_thd");
      variable_names.push_back("L_EB_thd");
      variable_names.push_back("L_WR_thd");
      variable_names.push_back("L_WFE_thd");
      variable_names.push_back("L_WAA_thd");
      variable_names.push_back("L_FR_thd");
      variable_names.push_back("L_RF_thd");
      variable_names.push_back("L_MF_thd");
      variable_names.push_back("L_LF_thd");
      variable_names.push_back("L_SFE_u");
      variable_names.push_back("L_SAA_u");
      variable_names.push_back("L_HR_u");
      variable_names.push_back("L_EB_u");
      variable_names.push_back("L_WR_u");
      variable_names.push_back("L_WFE_u");
      variable_names.push_back("L_WAA_u");
      variable_names.push_back("L_FR_u");
      variable_names.push_back("L_RF_u");
      variable_names.push_back("L_MF_u");
      variable_names.push_back("L_LF_u");
    }
    else if(specifications[i].class_name.compare("AudioRecorder") == 0)
    {
      const int NUM_AUDIO_SIGNALS = 28;
      for (int i = 0; i < NUM_AUDIO_SIGNALS; ++i)
      {
        variable_names.push_back(std::string("audio_") + usc_utilities::getString(i));
      }
    }

    else if(specifications[i].class_name.compare(RIGHT_ARM + "WrenchStatesRecorder") == 0)
    {
      variable_names.push_back(RIGHT_ARM_PREFIX + "palm_force_x");
      variable_names.push_back(RIGHT_ARM_PREFIX + "palm_force_y");
      variable_names.push_back(RIGHT_ARM_PREFIX + "palm_force_z");
      variable_names.push_back(RIGHT_ARM_PREFIX + "palm_torque_x");
      variable_names.push_back(RIGHT_ARM_PREFIX + "palm_torque_y");
      variable_names.push_back(RIGHT_ARM_PREFIX + "palm_torque_z");
    }
    else if(specifications[i].class_name.compare(LEFT_ARM + "WrenchStatesRecorder") == 0)
    {
      variable_names.push_back(LEFT_ARM_PREFIX + "palm_force_x");
      variable_names.push_back(LEFT_ARM_PREFIX + "palm_force_y");
      variable_names.push_back(LEFT_ARM_PREFIX + "palm_force_z");
      variable_names.push_back(LEFT_ARM_PREFIX + "palm_torque_x");
      variable_names.push_back(LEFT_ARM_PREFIX + "palm_torque_y");
      variable_names.push_back(LEFT_ARM_PREFIX + "palm_torque_z");
    }

    else if(specifications[i].class_name.compare(RIGHT_ARM + "StrainGaugeStatesRecorder") == 0)
    {
      variable_names.push_back(RIGHT_ARM_PREFIX + "RF_SG");
      variable_names.push_back(RIGHT_ARM_PREFIX + "MF_SG");
      variable_names.push_back(RIGHT_ARM_PREFIX + "LF_SG");
    }
    else if(specifications[i].class_name.compare(LEFT_ARM + "StrainGaugeStatesRecorder") == 0)
    {
      variable_names.push_back(LEFT_ARM_PREFIX + "RF_SG");
      variable_names.push_back(LEFT_ARM_PREFIX + "MF_SG");
      variable_names.push_back(LEFT_ARM_PREFIX + "LF_SG");
    }

    else if(specifications[i].class_name.compare(RIGHT_ARM + "PressureSensorStatesRecorder") == 0)
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
          variable_names.push_back(RIGHT_ARM_PREFIX + pressure_sensor_pad_names[i] + ss.str());
        }
      }
    }
    else if(specifications[i].class_name.compare(LEFT_ARM + "PressureSensorStatesRecorder") == 0)
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
          variable_names.push_back(LEFT_ARM_PREFIX + pressure_sensor_pad_names[i] + ss.str());
        }
      }
    }

    else if(specifications[i].class_name.compare(RIGHT_ARM + "AvgPressureSensorStatesRecorder") == 0)
    {
      variable_names.push_back(RIGHT_ARM_PREFIX + "avg_palm");
      variable_names.push_back(RIGHT_ARM_PREFIX + "avg_right_finger");
      variable_names.push_back(RIGHT_ARM_PREFIX + "avg_middle_finger");
      variable_names.push_back(RIGHT_ARM_PREFIX + "avg_left_finger");
    }
    else if(specifications[i].class_name.compare(LEFT_ARM + "AvgPressureSensorStatesRecorder") == 0)
    {
      variable_names.push_back(LEFT_ARM_PREFIX + "avg_palm");
      variable_names.push_back(LEFT_ARM_PREFIX + "avg_right_finger");
      variable_names.push_back(LEFT_ARM_PREFIX + "avg_middle_finger");
      variable_names.push_back(LEFT_ARM_PREFIX + "avg_left_finger");
    }

    else if(specifications[i].class_name.compare(RIGHT_ARM + "AccelerationsRecorder") == 0)
    {
      variable_names.push_back(RIGHT_ARM_PREFIX + "LC_ACC_X");
      variable_names.push_back(RIGHT_ARM_PREFIX + "LC_ACC_Y");
      variable_names.push_back(RIGHT_ARM_PREFIX + "LC_ACC_Z");
    }
    else if(specifications[i].class_name.compare(LEFT_ARM + "AccelerationsRecorder") == 0)
    {
      variable_names.push_back(LEFT_ARM_PREFIX + "LC_ACC_X");
      variable_names.push_back(LEFT_ARM_PREFIX + "LC_ACC_Y");
      variable_names.push_back(LEFT_ARM_PREFIX + "LC_ACC_Z");
    }

    else if(specifications[i].class_name.compare(RIGHT_ARM + "DistanceFeatureRecorder") == 0)
    {
      variable_names.push_back(RIGHT_ARM_PREFIX + "PALM");
      variable_names.push_back(RIGHT_ARM_PREFIX + "HAND_L_FINGER_TIP");
      variable_names.push_back(RIGHT_ARM_PREFIX + "HAND_M_FINGER_TIP");
      variable_names.push_back(RIGHT_ARM_PREFIX + "HAND_R_FINGER_TIP");
    }
    else if(specifications[i].class_name.compare(LEFT_ARM + "DistanceFeatureRecorder") == 0)
    {
      variable_names.push_back(LEFT_ARM_PREFIX + "PALM");
      variable_names.push_back(LEFT_ARM_PREFIX + "HAND_L_FINGER_TIP");
      variable_names.push_back(LEFT_ARM_PREFIX + "HAND_M_FINGER_TIP");
      variable_names.push_back(LEFT_ARM_PREFIX + "HAND_R_FINGER_TIP");
    }

    else
    {
      ROS_ASSERT_MSG(false, "Unknown class name >%s<. Cannot return all variable names.", specifications[i].class_name.c_str());
      return false;
    }
  }
  return true;
}


}


#endif /* TASK_RECORDER_SPECIFICATION_UTILITIES_H_ */
