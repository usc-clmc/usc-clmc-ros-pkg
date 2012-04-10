/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_event_detector.h

  \author	Peter Pastor
  \date		Jun 21, 2011

 *********************************************************************/

#ifndef TASK_EVENT_DETECTOR_H_
#define TASK_EVENT_DETECTOR_H_

// system includes
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <ros/ros.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_msgs/Description.h>

#include <task_recorder2_msgs/SetupDetector.h>
#include <task_recorder2_msgs/DetectedEvents.h>

#include <SafetyLight_msgs/SetColor.h>

// local includes
#include <task_event_detector/detector.h>
#include <task_event_detector/event_monitor.h>

namespace task_event_detector
{

// TODO: Fix the permission issue for different users writing to the same file
static const std::string LAST_DATA_SAMPLES_BAG_FILE_NAME = "/tmp/last_data_samples_bag_file.bag";
static const std::string LAST_DESCRIPTION_BAG_FILE_NAME = "/tmp/last_description_bag_file.bag";

static const std::string DATA_SAMPLE_TOPIC_NAME = "/TaskRecorderManager/data_samples";
static const std::string DESCRIPTION_TOPIC_NAME = "/TaskRecorderManager/description";

class TaskEventDetector : public Detector
{

public:

  /*! Constructor
   */
  TaskEventDetector(ros::NodeHandle node_handle = ros::NodeHandle("/TaskEventDetector"));
  /*! Destructor
   */
  virtual ~TaskEventDetector() {};

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  bool load(task_recorder2_msgs::SetupDetector::Request& request,
            task_recorder2_msgs::SetupDetector::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  bool detect(task_recorder2_msgs::DetectedEvents::Request& request,
              task_recorder2_msgs::DetectedEvents::Response& response);

private:

  /*!
   */
  ros::NodeHandle node_handle_;
  ros::Subscriber data_sample_subscriber_;
  ros::Publisher safety_light_publisher_;

  /*!
   */
  ros::ServiceServer load_service_server_;
  ros::ServiceServer detect_service_server_;

  /*!
   */
  EventMonitor event_monitor_;
  void dataSampleCB(const task_recorder2_msgs::DataSample& data_sample);

  /*!
   */
  task_recorder2_msgs::Description description_;

  /*!
   */
  boost::mutex mutex_;
  bool detecting_;

  bool save_data_samples_;
  std::vector<task_recorder2_msgs::DataSample> last_data_samples_;

  int num_labels_;
  bool data_labels_ready_;
  std::vector<task_recorder2_msgs::DataSampleLabel> last_data_labels_;
  boost::condition_variable cond_;
  boost::mutex sync_mutex_;

  /*!
   */
  void detect(const bool detect);
  bool isDetecting();

  /*!
   */
  void publish(const task_recorder2_msgs::DataSampleLabel& data_label);

};

}

#endif /* TASK_EVENT_DETECTOR_H_ */
