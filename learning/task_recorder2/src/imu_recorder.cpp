/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		imu_recorder.cpp

  \author	Peter Pastor
  \date		Jun 13, 2012

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>


// local includes
#include <task_recorder2/imu_recorder.h>

namespace task_recorder2
{

ImuRecorder::ImuRecorder(ros::NodeHandle node_handle) :
  TaskRecorder<sensor_msgs::Imu> (node_handle)
{
}

bool ImuRecorder::transformMsg(const sensor_msgs::Imu& imu_sample,
                                 task_recorder2_msgs::DataSample& data_sample)
{
  data_sample.header = imu_sample.header;
  // orientation
  data_sample.data[0] = imu_sample.orientation.w;
  data_sample.data[1] = imu_sample.orientation.x;
  data_sample.data[2] = imu_sample.orientation.y;
  data_sample.data[3] = imu_sample.orientation.z;
  // angular velocity
  data_sample.data[4] = imu_sample.angular_velocity.x;
  data_sample.data[5] = imu_sample.angular_velocity.y;
  data_sample.data[6] = imu_sample.angular_velocity.z;
  // linear acceleration
  data_sample.data[7] = imu_sample.linear_acceleration.x;
  data_sample.data[8] = imu_sample.linear_acceleration.y;
  data_sample.data[9] = imu_sample.linear_acceleration.z;
  return true;
}

std::vector<std::string> ImuRecorder::getNames() const
{
  // ROS_ASSERT_MSG(initialized_, "ImuRecorder is not initialize.");
  std::vector<std::string> names;
  names.push_back(std::string(arm_prefix_ + "_imu_qw"));
  names.push_back(std::string(arm_prefix_ + "_imu_qx"));
  names.push_back(std::string(arm_prefix_ + "_imu_qy"));
  names.push_back(std::string(arm_prefix_ + "_imu_qz"));
  names.push_back(std::string(arm_prefix_ + "_imu_angular_vel_x"));
  names.push_back(std::string(arm_prefix_ + "_imu_angular_vel_y"));
  names.push_back(std::string(arm_prefix_ + "_imu_angular_vel_z"));
  names.push_back(std::string(arm_prefix_ + "_imu_linear_acc_x"));
  names.push_back(std::string(arm_prefix_ + "_imu_linear_acc_y"));
  names.push_back(std::string(arm_prefix_ + "_imu_linear_acc_z"));
  return names;
}

}
