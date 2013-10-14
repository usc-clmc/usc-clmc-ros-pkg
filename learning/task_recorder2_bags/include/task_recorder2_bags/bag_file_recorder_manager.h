/*
 * bag_file_recorder_manager.h
 *
 *  Created on: Jun 10, 2013
 *      Author: pastor
 */

#ifndef BAG_FILE_RECORDER_MANAGER_H_
#define BAG_FILE_RECORDER_MANAGER_H_

#include <ros/ros.h>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <task_recorder2_bags/bag_file_recorder.h>
#include <task_recorder2_bags/bag_file_recorder_client.h>

namespace task_recorder2_bags
{

class BagFileRecorderManager
{

public:

  BagFileRecorderManager();
  virtual ~BagFileRecorderManager() {};

  bool initialize(ros::NodeHandle node_handle);

  bool startRecording(const std::string& description, const int id);
  bool stopRecording(const ros::Time& crop_start_time, const ros::Time& crop_end_time);

private:

  bool initialized_;
  std::vector<boost::shared_ptr<task_recorder2_bags::BagFileRecorderBase> > clients_;

};

}

#endif /* BAG_FILE_RECORDER_MANAGER_H_ */
