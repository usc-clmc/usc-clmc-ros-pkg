/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		imu_recorder.h

  \author	Peter Pastor
  \date		Jun 13, 2012

 *********************************************************************/

#ifndef IMU_RECORDER_H_
#define IMU_RECORDER_H_

// system includes
#include <vector>
#include <string>

#include <task_recorder2_msgs/DataSample.h>

#include <sensor_msgs/Imu.h>

// local includes
#include <task_recorder2/task_recorder.h>

namespace task_recorder2_recorders
{

class ImuRecorder : public task_recorder2::TaskRecorder<sensor_msgs::Imu>
{

public:

  /*! Constructor
   * @param node_handle
   */
  ImuRecorder(ros::NodeHandle node_handle) {};
  /*! Destructor
   */
  virtual ~ImuRecorder() {};

  /*!
   * @return True on success, otherwise False
   */
  bool initialize(const std::string topic_name = std::string("/ImuRecorder"))
  {
    return task_recorder2::TaskRecorder<sensor_msgs::Imu>::initialize(topic_name);
  }

  /*!
   * @param imu_sample
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool transformMsg(const sensor_msgs::ImuConstPtr imu_sample,
                    task_recorder2_msgs::DataSample& data_sample);

  /*!
   * @return
   */
  unsigned int getNumSignals() const
  {
    // quaternion + angular velocity + linear_acceleration
    return 4 + 3 + 3;
  }

  /*!
   * @return
   */
  std::vector<std::string> getNames() const;

  /*!
   * @return
   */
  static std::string getClassName()
  {
    return std::string("ImuRecorder");
  }

private:

};


}


#endif /* IMU_RECORDER_H_ */
