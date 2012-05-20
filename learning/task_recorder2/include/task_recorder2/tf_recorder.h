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

#include <tf/tfMessage.h>

// local includes
#include <task_recorder2/task_recorder.h>
#include <task_recorder2_msgs/DataSample.h>

namespace task_recorder2
{

class TFRecorder : public TaskRecorder<tf::tfMessage>
{

public:

  /*! Constructor
   */
  TFRecorder(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~TFRecorder() {};

  /*!
   * @return True on success, otherwise False
   */
  bool initialize(const std::string topic_name = std::string("/tf"))
  {
    return TaskRecorder<tf::tfMessage>::initialize(topic_name);
  }

  /*!
   * @param transform
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool transformMsg(const tf::tfMessage& transform,
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

  int num_transforms_;
  std::vector<std::string> transform_names_;
  std::vector<int> indices_;

};

}

#endif /* TF_RECORDER_H_ */
