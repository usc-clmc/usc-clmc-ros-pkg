/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		label_gui_client.h

  \author	Peter Pastor
  \date		Aug 5, 2011

 *********************************************************************/

#ifndef LABEL_GUI_CLIENT_H_
#define LABEL_GUI_CLIENT_H_

// system includes
#include <ros/ros.h>
#include <vector>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes

namespace task_label_gui
{

class LabelGuiClient
{

public:

  /*! Constructor
   */
  LabelGuiClient(ros::NodeHandle node_handle = ros::NodeHandle("/LabelGui"));
  /*! Destructor
   */
  virtual ~LabelGuiClient() {};

  /*!
   * @param descriptions
   * @param data_sample_labels
   * @return True on success and if ARM_LABEL_GRASPS is set, otherwise False
   */
  bool setLabel(const std::vector<task_recorder2_msgs::Description>& descriptions,
                std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels,
                const bool wait_for_label_client = true);
  bool setLabel(const task_recorder2_msgs::Description& description,
                task_recorder2_msgs::DataSampleLabel& data_sample_label,
                const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::Description> descriptions;
    descriptions.push_back(description);
    std::vector<task_recorder2_msgs::DataSampleLabel> data_sample_labels;
    data_sample_labels.push_back(data_sample_label);
    return setLabel(descriptions, data_sample_labels, wait_for_label_client);
  }
  bool setLabel(const std::vector<task_recorder2_msgs::Description>& descriptions,
                const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::DataSampleLabel> data_sample_labels;
    for (int i = 0; i < (int)descriptions.size(); ++i)
    {
      task_recorder2_msgs::DataSampleLabel data_sample_label;
      data_sample_labels.push_back(data_sample_label);
    }
    return setLabel(descriptions, data_sample_labels, wait_for_label_client);
  }
  bool setLabel(const task_recorder2_msgs::Description& description,
                const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::Description> descriptions;
    descriptions.push_back(description);
    return setLabel(descriptions, wait_for_label_client);
  }

  /*!
   * @param descriptions
   * @param data_sample_labels
   * @return True on success and if ARM_LABEL_GRASPS is set, otherwise False
   */
  bool recordAndLabel(const std::vector<task_recorder2_msgs::Description>& descriptions,
                      std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels,
                      const bool wait_for_label_client = true);
  bool recordAndLabel(const task_recorder2_msgs::Description& description,
                      task_recorder2_msgs::DataSampleLabel& data_sample_label,
                      const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::Description> descriptions;
    descriptions.push_back(description);
    std::vector<task_recorder2_msgs::DataSampleLabel> data_sample_labels;
    data_sample_labels.push_back(data_sample_label);
    return recordAndLabel(descriptions, data_sample_labels, wait_for_label_client);
  }
  bool recordAndLabel(const std::vector<task_recorder2_msgs::Description>& descriptions,
                      const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::DataSampleLabel> data_sample_labels;
    for (int i = 0; i < (int)descriptions.size(); ++i)
    {
      task_recorder2_msgs::DataSampleLabel data_sample_label;
      data_sample_labels.push_back(data_sample_label);
    }
    return recordAndLabel(descriptions, data_sample_labels, wait_for_label_client);
  }
  bool recordAndLabel(const task_recorder2_msgs::Description& description,
                      const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::Description> descriptions;
    descriptions.push_back(description);
    return recordAndLabel(descriptions, wait_for_label_client);
  }

  /*!
   * @param descriptions
   * @return True on success and if ARM_LABEL_GRASPS is set, otherwise False
   */
  bool setTrialIds(std::vector<task_recorder2_msgs::Description>& descriptions,
                   const bool wait_for_label_client = true);
  bool setTrialIds(task_recorder2_msgs::Description& description,
                   const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::Description> descriptions;
    descriptions.push_back(description);
    return setTrialIds(descriptions, wait_for_label_client);
  }

private:

  ros::NodeHandle node_handle_;
  ros::ServiceClient set_last_trial_ids_service_client_;
  ros::ServiceClient set_label_service_client_;
  ros::ServiceClient record_and_label_service_client_;

};

}


#endif /* LABEL_GUI_CLIENT_H_ */
