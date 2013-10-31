/*
 * pose_recorder.h
 *
 *  Created on: Jul 20, 2011
 *      Author: schorfi
 */

#ifndef POSE_RECORDER_H_
#define POSE_RECORDER_H_

// system includes
#include <vector>
#include <string>

#include <geometry_msgs/PoseStamped.h>

// local includes
#include <task_recorder2/task_recorder.h>
#include <task_recorder2_msgs/DataSample.h>

namespace task_recorder2_recorders
{

class PoseRecorder : public task_recorder2::TaskRecorder<geometry_msgs::PoseStamped>
{

public:
  /*! Constructor
   * @param node_handle
   */
  PoseRecorder(ros::NodeHandle node_handle) {};
  /*! Destructor
   */
  virtual ~PoseRecorder() {};

  /*!
   * @return True on success, otherwise False
   */
  bool initialize(const std::string topic_name)
  {
    return task_recorder2::TaskRecorder<geometry_msgs::PoseStamped>::initialize(topic_name);
  }

  /*!
   * @param pose
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool transformMsg(const geometry_msgs::PoseStampedConstPtr pose,
                    task_recorder2_msgs::DataSample& data_sample);

  /*!
   * @return
   */
  unsigned int getNumSignals() const
  {
    return NUM_POSE_VARIABLES;
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
    return std::string("PoseRecorder");
  }

private:

  static const unsigned int NUM_POSE_VARIABLES = 7;
  enum
  {
    POSITION_X = 0, POSITION_Y, POSITION_Z, POSITION_QW, POSITION_QX, POSITION_QY, POSITION_QZ
  };

};

}

#endif /* POSE_RECORDER_H_ */
