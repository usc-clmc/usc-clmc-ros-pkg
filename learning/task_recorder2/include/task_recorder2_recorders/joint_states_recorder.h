/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    joint_states_recorder.h

 \author  Peter Pastor
 \date    Jun 21, 2010

 *********************************************************************/

#ifndef JOINT_STATES_RECORDER_H_
#define JOINT_STATES_RECORDER_H_

// system includes
#include <vector>
#include <string>

#include <sensor_msgs/JointState.h>

// local includes
#include <task_recorder2/task_recorder.h>
#include <task_recorder2_msgs/DataSample.h>

namespace task_recorder2_recorders
{

class JointStatesRecorder : public task_recorder2::TaskRecorder<sensor_msgs::JointState>
{

public:

  /*! Constructor
   */
  JointStatesRecorder(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~JointStatesRecorder() {};

  /*!
   * @return True on success, otherwise False
   */
  bool initialize(const std::string topic_name = std::string("/joint_states"))
  {
    return task_recorder2::TaskRecorder<sensor_msgs::JointState>::initialize(topic_name);
  }

  /*!
   * @param joint_state
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool transformMsg(const sensor_msgs::JointStateConstPtr joint_state,
                    task_recorder2_msgs::DataSample& data_sample);

  /*!
   * @return
   */
  unsigned int getNumSignals() const
  {
    return POS_VEL_EFF * joint_names_.size();
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
    return std::string("JointStatesRecorder");
  }

private:

  static const unsigned int POS_VEL_EFF = 3;
  static const unsigned int POSITIONS = 0;
  static const unsigned int VELOCITIES = 1;
  static const unsigned int EFFORTS = 2;

  unsigned int num_joint_states_;
  std::vector<std::string> joint_names_;
  std::vector<int> indices_;

};

}

#endif /* JOINT_STATES_RECORDER_H_ */
