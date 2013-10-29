/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_recorder_manager.h

  \author	Peter Pastor
  \date		Jun 6, 2011

 *********************************************************************/

#ifndef TASK_RECORDER_MANAGER_H_
#define TASK_RECORDER_MANAGER_H_

// system includes
#include <vector>
#include <map>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <task_recorder2_msgs/DataSample.h>

#include <task_recorder2_recorders/joint_states_recorder.h>
#include <task_recorder2_recorders/audio_recorder.h>

#include <task_recorder2/task_recorder_client.h>

#include <task_recorder2_srvs/StartStreaming.h>
#include <task_recorder2_srvs/StopStreaming.h>
#include <task_recorder2_srvs/StartRecording.h>
#include <task_recorder2_srvs/StopRecording.h>
#include <task_recorder2_srvs/InterruptRecording.h>
#include <task_recorder2_srvs/GetInfo.h>
#include <task_recorder2_srvs/GetDataSample.h>
#include <task_recorder2_srvs/AddDataSamples.h>
#include <task_recorder2_srvs/ReadDataSamples.h>

#include <task_recorder2_msgs/TaskRecorderSpecification.h>
#include <task_recorder2_io/task_recorder_io.h>

#include <blackboard/blackboard_client.h>

// local includes
#include <task_recorder2/task_recorder.h>
#include <task_recorder2/interrupt_handler.h>

namespace task_recorder2
{

class TaskRecorderManager : public InterruptHandler
{

public:

  /*! Constructor
   * @param node_handle
   */
  TaskRecorderManager(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~TaskRecorderManager();

  /*!
   * @return True on success, False otherwise
   */
  bool initialize();

  /*!
   * @param task_recorders
   * @return True on success, otherwise False
   */
  virtual bool read(std::vector<boost::shared_ptr<task_recorder2::TaskRecorderBase> >& task_recorders)
  {
    return true;
  }

  /*! Starts the spinner
   */
  void run();

  /*!
   * @return the number of used task recorders
   */
  unsigned int getNumberOfTaskRecorders() const;

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool startStreaming(task_recorder2_srvs::StartStreaming::Request& request,
                      task_recorder2_srvs::StartStreaming::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool stopStreaming(task_recorder2_srvs::StopStreaming::Request& request,
                     task_recorder2_srvs::StopStreaming::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool startRecording(task_recorder2_srvs::StartRecording::Request& request,
                      task_recorder2_srvs::StartRecording::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool stopRecording(task_recorder2_srvs::StopRecording::Request& request,
                     task_recorder2_srvs::StopRecording::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool interruptRecording(task_recorder2_srvs::InterruptRecording::Request& request,
                          task_recorder2_srvs::InterruptRecording::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool getInfo(task_recorder2_srvs::GetInfo::Request& request,
               task_recorder2_srvs::GetInfo::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool getDataSample(task_recorder2_srvs::GetDataSample::Request& request,
                     task_recorder2_srvs::GetDataSample::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool addDataSamples(task_recorder2_srvs::AddDataSamples::Request& request,
                      task_recorder2_srvs::AddDataSamples::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, False otherwise
   */
  bool readDataSamples(task_recorder2_srvs::ReadDataSamples::Request& request,
                       task_recorder2_srvs::ReadDataSamples::Response& response);

protected:

  /*!
   */
  task_recorder2_io::TaskRecorderIO<task_recorder2_msgs::DataSample> recorder_io_;

private:

  double sampling_rate_;
  ros::Timer timer_;
  unsigned int counter_;

  /*! task recorders
   */
  std::vector<boost::shared_ptr<task_recorder2::TaskRecorderBase> > task_recorders_;
  /*! data samples
   */
  std::vector<task_recorder2_msgs::DataSample> data_samples_;

  task_recorder2_msgs::DataSample last_combined_data_sample_;
  boost::mutex last_combined_data_sample_mutex_;

  /*!
   */
  std::vector<task_recorder2_srvs::StartStreaming::Request> start_streaming_requests_;
  std::vector<task_recorder2_srvs::StartStreaming::Response> start_streaming_responses_;
  std::vector<task_recorder2_srvs::StopStreaming::Request> stop_streaming_requests_;
  std::vector<task_recorder2_srvs::StopStreaming::Response> stop_streaming_responses_;
  std::vector<task_recorder2_srvs::StartRecording::Request> start_recording_requests_;
  std::vector<task_recorder2_srvs::StartRecording::Response> start_recording_responses_;
  std::vector<task_recorder2_srvs::StopRecording::Request> stop_recording_requests_;
  std::vector<task_recorder2_srvs::StopRecording::Response> stop_recording_responses_;
  std::vector<task_recorder2_srvs::InterruptRecording::Request> interrupt_recording_requests_;
  std::vector<task_recorder2_srvs::InterruptRecording::Response> interrupt_recording_responses_;

  /*!
   */
  ros::Publisher data_sample_publisher_;
  ros::ServiceServer start_streaming_service_server_;
  ros::ServiceServer stop_streaming_service_server_;
  ros::ServiceServer start_recording_service_server_;
  ros::ServiceServer stop_recording_service_server_;
  ros::ServiceServer interrupt_recording_service_server_;
  ros::ServiceServer get_info_service_server_;
  ros::ServiceServer get_data_sample_service_server_;
  ros::ServiceServer add_data_samples_service_server_;
  ros::ServiceServer read_data_samples_service_server_;

  /*!
   */
  // mutex to protect access to services
  boost::mutex service_mutex_;

  boost::shared_ptr<ros::AsyncSpinner> async_spinner_;

  /*!
   */
  ros::Publisher stop_recording_publisher_;

  /*!
   * @param timer_event
   */
  void timerCB(const ros::TimerEvent& timer_event);

  /*!
   * @param time_stamp
   * @return True if last sample is updated, otherwise False
   */
  bool setLastDataSample(const ros::Time& time_stamp);

  /*! visualize status info
   */
  blackboard::RightBlackBoardClient blackboard_client_;
  void updateInfo();

};

}

#endif /* TASK_RECORDER_MANAGER_H_ */
