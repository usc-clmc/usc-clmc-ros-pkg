/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_event_detector_client.h

  \author	Peter Pastor
  \date		Jul 8, 2011

 *********************************************************************/

#ifndef TASK_EVENT_DETECTOR_CLIENT_H_
#define TASK_EVENT_DETECTOR_CLIENT_H_

// system includes
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_client/task_recorder_manager_client.h>

#include <task_label_gui/label_gui_client.h>

// local includes
#include <task_event_detector/task_event_detector.h>
#include <task_event_detector/detector.h>

namespace task_event_detector
{

class TaskEventDetectorClient : public Detector
{

public:

  /*! Constructor
   */
  TaskEventDetectorClient(ros::NodeHandle node_handle = ros::NodeHandle("TaskEventDetectorClient"));

  /*! Destructor
   */
  virtual ~TaskEventDetectorClient() {};

  /*!
   * @param description
   * @return True if SVM exists, otherwise False
   */
  bool doesSVMExist(const task_recorder2_msgs::Description& description);

  /*!
   * @param description_string
   * @param id (default is 0)
   * @return True if SVM exists, otherwise False
   */
  bool doesSVMExist(const std::string& description_string,
                    const int id = 0)
  {
    task_recorder2_msgs::Description description;
    description.description = description_string;
    description.id = id;
    return doesSVMExist(description);
  }

  /*!
   * @param description
   * @return True on success, otherwise False
   */
  bool setDescription(const task_recorder2_msgs::Description& description);

  /*!
   * @param description_string
   * @param id (default is 0)
   * @return True on success, otherwise False
   */
  bool setDescription(const std::string description_string,
                      const int id = 0)
  {
    task_recorder2_msgs::Description description;
    description.description = description_string;
    description.id = id;
    return setDescription(description);
  }

  /*!
   * @param svm_description
   * @param data_samples
   * @param predicted_labels
   * @return True on success, otherwise False
   */
  bool getLabels(const task_recorder2_msgs::Description& svm_description,
                 const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                 std::vector<task_recorder2_msgs::DataSampleLabel>& predicted_labels);

  /*!
   * @param label
   * @param save_data_samples
   * @param num_data_samples
   * @return True on success, otherwise False
   */
  bool checkForEvent(task_recorder2_msgs::DataSampleLabel& label,
                     const bool save_data_samples = true,
                     const int num_data_samples = 100);

  /*!
   * @param event_occured True if event occured, otherwise False
   * @param save_data_samples
   * @param num_data_samples
   * @return False if internal error occured, otherwise True
   */
  bool didEventOccur(bool& event_occured,
                     const bool save_data_samples = true,
                     const int num_samples = 100);

  /*!
   * @param descriptions
   * @param data_sample_labels
   * @param wait_for_label_client
   * @return True on success, otherwise False
   */
  bool labelSample(const std::vector<task_recorder2_msgs::Description>& descriptions,
                   std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels,
                   const bool wait_for_label_client = true);
  /*!
   * @param description
   * @param data_sample_label
   * @param wait_for_label_client
   * @return True on success, otherwise False
   */
  bool labelSample(const task_recorder2_msgs::Description& description,
                   task_recorder2_msgs::DataSampleLabel& data_sample_label,
                   const bool wait_for_label_client = true)
  {
    std::vector<task_recorder2_msgs::Description> descriptions;
    descriptions.push_back(description);
    std::vector<task_recorder2_msgs::DataSampleLabel> data_sample_labels;
    data_sample_labels.push_back(data_sample_label);
    return labelSample(descriptions, data_sample_labels, wait_for_label_client);
  }

  /*!
   * @param description
   * @param wait_for_label_client
   * @return True on success, otherwise False
   */
  bool labelSample(const task_recorder2_msgs::Description& description,
                   const bool wait_for_label_client = true)
  {
    task_recorder2_msgs::DataSampleLabel data_sample_label;
    return labelSample(description, data_sample_label, wait_for_label_client);
  }

  /*!
   * @param description_string
   * @param data_sample_label
   * @param id
   * @param wait_for_label_client
   * @return True on success, otherwise False
   */
  bool labelSample(const std::string& description_string,
                   task_recorder2_msgs::DataSampleLabel& data_sample_label,
                   const int id = 0,
                   const bool wait_for_label_client = true)
  {
    task_recorder2_msgs::Description description;
    description.description = description_string;
    description.id = id;
    return labelSample(description, data_sample_label, wait_for_label_client);
  }

  /*!
   * @param description_string
   * @param id
   * @return True on success, otherwise False
   */
  bool labelSample(const std::string& description_string,
                   const int id = 0,
                   const bool wait_for_label_client = true)
  {
    task_recorder2_msgs::Description description;
    description.description = description_string;
    description.id = id;
    return labelSample(description, wait_for_label_client);
  }

  /*!
   * @return True on success, otherwise False
   */
  bool labelSample()
  {
    if(!initialized_)
    {
      ROS_ERROR("No description set. Cannot label sample. You need to specify a description.");
      return false;
    }
    return labelSample(description_);
  }

  /*!
   * @param description
   * @param data_samples
   * @return True on success, otherwise False
   */
  bool addDataSamples(const task_recorder2_msgs::Description& description,
                      const std::vector<task_recorder2_msgs::DataSample>& data_samples)
  {
    return task_recorder_manager_client_.addDataSamples(description, data_samples);
  }

private:

  bool initialized_;
  ros::NodeHandle node_handle_;

  /*!
   */
  ros::ServiceClient load_service_client_;
  ros::ServiceClient detect_service_client_;

  /*!
   */
  task_recorder2_msgs::Description description_;
  task_recorder2_client::TaskRecorderManagerClient task_recorder_manager_client_;
  task_label_gui::LabelGuiClient label_gui_client_;

};

typedef boost::shared_ptr<TaskEventDetectorClient> TaskEventDetectorClientPtr;

}


#endif /* TASK_EVENT_DETECTOR_CLIENT_H_ */
