/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks       ...
 
 \file      task_event_detector.cpp

 \author    Peter Pastor
 \date      Jun 21, 2011

 *********************************************************************/

// system includes
#include <vector>
#include <boost/shared_ptr.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_utilities/data_sample_label_utilities.h>

// local includes
#include <task_event_detector/task_event_detector.h>

namespace task_event_detector
{

TaskEventDetector::TaskEventDetector(ros::NodeHandle node_handle) :
  node_handle_(node_handle), detecting_(false), save_data_samples_(false)
{
  event_monitor_.registerCallback(boost::bind(&TaskEventDetector::dataSampleCB, this, _1));
  const int PUBLISHER_BUFFER_SIZE = 10;
  safety_light_publisher_ = node_handle_.advertise<SafetyLight_msgs::SetColor>("/SafetyLight/set_color", PUBLISHER_BUFFER_SIZE);
  load_service_server_ = node_handle_.advertiseService("load", &TaskEventDetector::load, this);
  detect_service_server_ = node_handle_.advertiseService("detect", &TaskEventDetector::detect, this);
}

void TaskEventDetector::detect(const bool detect)
{
  boost::mutex::scoped_lock(mutex_);
  detecting_ = detect;
}

bool TaskEventDetector::isDetecting()
{
  bool detecting = false;
  boost::mutex::scoped_lock(mutex_);
  detecting = detecting_;
  return detecting;
}

void TaskEventDetector::dataSampleCB(const task_recorder2_msgs::DataSample& data_sample)
{
  boost::mutex::scoped_lock(mutex_);
  if (detecting_)
  {
    task_recorder2_msgs::DataSampleLabel data_label;
    if(!predict(data_sample, data_label))
    {
      ROS_ERROR("Could not predict.");
      return;
    }
    publish(data_label);

    if(save_data_samples_)
    {
      last_data_samples_.push_back(data_sample);
    }

    if ((int)last_data_labels_.size() < num_labels_)
    {
      last_data_labels_.push_back(data_label);
      if ((int)last_data_labels_.size() >= num_labels_)
      {
        boost::lock_guard<boost::mutex> lock(sync_mutex_);
        data_labels_ready_ = true;
        cond_.notify_one();
      }
    }
  }
}

bool TaskEventDetector::detect(task_recorder2_msgs::DetectedEvents::Request& request,
                               task_recorder2_msgs::DetectedEvents::Response& response)
{
  mutex_.lock();
  last_data_labels_.clear();
  last_data_samples_.clear();
  num_labels_ = request.num_data_samples;
  save_data_samples_ = request.save_data_samples;
  bool has_been_detecting = detecting_;
  detecting_ = true;
  mutex_.unlock();

  sync_mutex_.lock();
  data_labels_ready_ = false;
  sync_mutex_.unlock();

  // gather data
  boost::unique_lock<boost::mutex> lock(sync_mutex_);
  while (!data_labels_ready_)
  {
    cond_.wait(lock);
  }

  response.labels = last_data_labels_;
  if(!computeLabel(request.method, response.labels, response.label))
  {
    response.return_code = task_recorder2_msgs::DetectedEvents::Response::SERVICE_CALL_FAILED;
    response.info.assign("Could not compute label.");
    return true;
  }

  mutex_.lock();
  detecting_ = has_been_detecting;
  save_data_samples_ = false;
  mutex_.unlock();

  if(request.save_data_samples)
  {
    ROS_WARN_COND(request.num_data_samples != (int)last_data_samples_.size(),
                  "Number of data samples >%i< does not correspond to number of recorded samples >%i<. Number of labels is >%i<.",
                  request.num_data_samples, (int)last_data_samples_.size(), (int)last_data_labels_.size());
    // offset time stamps
    if(last_data_samples_.empty())
    {
      response.return_code = task_recorder2_msgs::DetectedEvents::Response::SERVICE_CALL_FAILED;
      response.info.assign("No data samples recorded. Cannot save last data samples.");
      return true;
    }
    ros::Time start_time = last_data_samples_[0].header.stamp;
    for (int i = 0; i < (int)last_data_samples_.size(); ++i)
    {
      last_data_samples_[i].header.stamp = static_cast<ros::Time> (ros::TIME_MIN + (last_data_samples_[i].header.stamp - start_time));
    }
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSample>::writeToBagFileWithTimeStamps(last_data_samples_, DATA_SAMPLE_TOPIC_NAME, LAST_DATA_SAMPLES_BAG_FILE_NAME));
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::Description>::writeToBagFileWithTimeStamp(request.description, ros::TIME_MIN, DESCRIPTION_TOPIC_NAME, LAST_DESCRIPTION_BAG_FILE_NAME));
  }

  response.return_code = task_recorder2_msgs::DetectedEvents::Response::SERVICE_CALL_SUCCESSFUL;
  response.info.assign("Detection result is " + boost::lexical_cast<std::string>(response.label.binary_label.label) + ".");
  return true;
}

bool TaskEventDetector::load(task_recorder2_msgs::SetupDetector::Request& request,
                             task_recorder2_msgs::SetupDetector::Response& response)
{
  if(isDetecting())
  {
    response.return_code = task_recorder2_msgs::SetupDetector::Response::SERVICE_CALL_FAILED;
    response.info.assign("Could not load SVM from description " + task_recorder2_utilities::getFileName(request.description) + ". SVM is in use.");
    return true;
  }
  if(!Detector::load(request.description))
  {
    response.return_code = task_recorder2_msgs::SetupDetector::Response::SERVICE_CALL_FAILED;
    response.info.assign("Could not load SVM from description " + task_recorder2_utilities::getFileName(request.description) + ".");
    return true;
  }
  description_ = request.description;
  response.return_code = task_recorder2_msgs::SetupDetector::Response::SERVICE_CALL_SUCCESSFUL;
  response.info.assign("Succesfully loaded SVM from description " + task_recorder2_utilities::getFileName(request.description) + ".");
  return true;
}

void TaskEventDetector::publish(const task_recorder2_msgs::DataSampleLabel& data_label)
{
  SafetyLight_msgs::SetColor set_color;
  set_color.r = 255 * (1-data_label.binary_label.label);
  set_color.g = 255 * data_label.binary_label.label;
  set_color.b = 0;
  safety_light_publisher_.publish(set_color);

  ROS_INFO_COND(data_label.binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED,
                "Classification result is FAILED    (%.2f).", data_label.cost_label.cost);
  ROS_INFO_COND(data_label.binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED,
                "Classification result is SUCCEEDED (%.2f).", data_label.cost_label.cost);
}

}
