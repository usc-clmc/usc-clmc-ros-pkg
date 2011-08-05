/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_description_utilities.h

  \author	Peter Pastor
  \date		Jun 18, 2011

 *********************************************************************/

#ifndef TASK_DESCRIPTION_UTILITIES_H_
#define TASK_DESCRIPTION_UTILITIES_H_

// system includes
#include <string>
#include <map>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes
#include <task_recorder2_utilities/task_recorder_utilities.h>

namespace task_recorder2_utilities
{

inline int getId(const task_recorder2_msgs::Description& description);
inline std::string getStringId(const task_recorder2_msgs::Description& description);
inline std::string getDescription(const task_recorder2_msgs::Description& description);
inline std::string getFileName(const task_recorder2_msgs::Description& description);
inline std::string getBagFileName(const task_recorder2_msgs::Description& description);

inline bool check(std::string description);
inline bool check(int label_type);

inline bool readDescriptionLabelTypeMap(ros::NodeHandle node_handle,
                                        std::map<std::string, int>& description_label_type_map);
inline bool readDescriptionLabels(ros::NodeHandle node_handle,
                                  std::vector<std::string>& description_labels);

inline int getId(const task_recorder2_msgs::Description& description)
{
  return description.id;
}

inline std::string getStringId(const task_recorder2_msgs::Description& description)
{
  return task_recorder2_utilities::getString(description.id);
}

inline std::string getDescription(const task_recorder2_msgs::Description& description)
{
  return description.description;
}

inline std::string getFileName(const task_recorder2_msgs::Description& description)
{
  return getDescription(description) + FILE_NAME_ID_SEPARATOR + getStringId(description);
}

inline std::string getBagFileName(const task_recorder2_msgs::Description& description)
{
  return getDescription(description) + FILE_NAME_ID_SEPARATOR + getStringId(description) + BAG_FILE_APPENDIX;
}

inline bool check(std::string description)
{
  return (description.compare(task_recorder2_msgs::Description::SIDE_GRASP) == 0
      || description.compare(task_recorder2_msgs::Description::TOP_GRASP) == 0
      || description.compare(task_recorder2_msgs::Description::PLACING) == 0
      || description.compare(task_recorder2_msgs::Description::RELEASING) == 0
      || description.compare(task_recorder2_msgs::Description::TURN_ON_DRILL) == 0
      || description.compare(task_recorder2_msgs::Description::DRILLING) == 0);
}

inline bool check(int label_type)
{
  return (label_type == task_recorder2_msgs::DataSampleLabel::BINARY_LABEL
      || label_type == task_recorder2_msgs::DataSampleLabel::COST_LABEL);
}

inline bool readDescriptionLabels(ros::NodeHandle node_handle,
                                  std::vector<std::string>& description_labels)
{
  std::map<std::string, int> description_label_type_map;
  ROS_VERIFY(readDescriptionLabelTypeMap(node_handle, description_label_type_map));
  if(description_label_type_map.empty())
  {
    ROS_ERROR("Description label type map is empty. Problems while reading from node handle in namespace >%s<.", node_handle.getNamespace().c_str());
    return false;
  }
  description_labels.clear();
  std::map<std::string, int>::const_iterator ci;
  for(ci = description_label_type_map.begin(); ci != description_label_type_map.end(); ++ci)
  {
    description_labels.push_back(ci->first);
  }
  return true;
}

inline bool readDescriptionLabelTypeMap(ros::NodeHandle node_handle,
                                        std::map<std::string, int>& description_label_type_map)
{
  description_label_type_map.clear();
  // read the list of description-label_type mapping from the param server
  XmlRpc::XmlRpcValue xml_description_label_type_map;
  if (!node_handle.getParam("description_label_type_map", xml_description_label_type_map))
  {
    ROS_ERROR("Couldn't find parameter >%s/description_label_type_map<", node_handle.getNamespace().c_str());
    return false;
  }
  if (xml_description_label_type_map.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR(">%s/description_label_type_map must be a struct and not of type >%i<.",
        node_handle.getNamespace().c_str(), (int)xml_description_label_type_map.getType());
    return false;
  }

  for (int i = 0; i < xml_description_label_type_map.size(); ++i)
  {
    if (!xml_description_label_type_map[i].hasMember("description"))
    {
      ROS_ERROR("Description-LabelType map must have a field \"description\".");
      return false;
    }
    std::string description = xml_description_label_type_map[i]["description"];
    ROS_VERIFY_MSG(task_recorder2_utilities::check(description), "Invalid description >%s<.", description.c_str());

    if (!xml_description_label_type_map[i].hasMember("label_type"))
    {
      ROS_ERROR("Description-LabelType map must have a field \"label_type\".");
      return false;
    }
    int label_type = xml_description_label_type_map[i]["label_type"];
    ROS_VERIFY_MSG(task_recorder2_utilities::check(label_type), "Invalid description >%i<.", label_type);

    description_label_type_map.insert(std::pair<std::string, int>(description, label_type));
    ROS_DEBUG("Mapping description >%s< onto label type >%i<.", description.c_str(), label_type);
  }
  return true;
}

}

#endif /* TASK_DESCRIPTION_UTILITIES_H_ */
