/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    tf_recorder.h

 \author  Peter Pastor
 \date    Dec 8, 2011

 *********************************************************************/


#ifndef TF_RECORDER_H_
#define TF_RECORDER_H_

// system includes
#include <vector>
#include <string>

#include <tf/transform_listener.h>

// local includes
#include <task_recorder2/task_recorder.h>
#include <task_recorder2_msgs/DataSample.h>

namespace task_recorder2_recorders
{

class TFRecorder : public task_recorder2::TaskRecorder<task_recorder2_msgs::DataSample>
{

public:

  /*! Constructor
   * @param node_handle
   */
  TFRecorder(ros::NodeHandle node_handle) {};
  /*! Destructor
   */
  virtual ~TFRecorder() {};

  /*! Read tf recorder specific parameters
   * @param node_handle
   * @return True on success otherwise False
   */
  bool readParams(ros::NodeHandle& node_handle);

  /*!
   * @param msg (empty)
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool transformMsg(const task_recorder2_msgs::DataSample& msg,
                    task_recorder2_msgs::DataSample& data_sample);

  /*!
   * @return
   */
  int getNumSignals() const
  {
    return static_cast<int>(NUM_SIGNALS_PER_TRANSFORM * transform_names_.size());
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
    return std::string("TFRecorder");
  }

private:

  static const int NUM_SIGNALS_PER_TRANSFORM = (3+4);

  std::vector<std::string> transform_names_;
  std::vector<int> indices_;

  tf::TransformListener tf_listener_;
};

}

#endif /* TF_RECORDER_H_ */
