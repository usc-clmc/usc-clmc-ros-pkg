/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    tf_recorder.cpp

 \author  Peter Pastor
 \date    Dec 8, 2011

 *********************************************************************/


// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_utilities/task_recorder_utilities.h>

// local includes
#include <task_recorder2/tf_recorder.h>

namespace task_recorder2
{

TFRecorder::TFRecorder(ros::NodeHandle node_handle) :
  TaskRecorder<tf::tfMessage> (node_handle), num_transforms_(0)
{
  ROS_VERIFY(usc_utilities::read(node_handle, "transforms", transform_names_));
  ROS_DEBUG("Starting tf recorder for transform:");
  for (int i = 0; i < (int)transform_names_.size(); ++i)
  {
    ROS_DEBUG("%i) %s", i+1, transform_names_[i].c_str());
  }
}

bool TFRecorder::transformMsg(const tf::tfMessage& transform,
                              task_recorder2_msgs::DataSample& data_sample)
{
  if(first_time_)
  {
    std::vector<std::string> msg_transform_names;
    for (int i = 0; i < (int)transform.transforms.size(); ++i)
    {
      msg_transform_names.push_back(transform.transforms[i].child_frame_id);
    }
    ROS_VERIFY(task_recorder2_utilities::getIndices(msg_transform_names, transform_names_, indices_));
    num_transforms_ = (int)transform_names_.size();
  }
  ROS_ASSERT_MSG(!transform.transforms.empty(), "No transform msgs contained. Cannot transform msg. Cannot initialize tf recorder.");
  data_sample.header = transform.transforms[0].header;

  ROS_ASSERT((int)data_sample.data.size() == (num_transforms_ * NUM_SIGNALS_PER_TRANSFORM));
  // positions, velicities, and acceleration
  for (int i = 0; i < num_transforms_; ++i)
  {
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 0] = transform.transforms[indices_[i]].transform.translation.x;
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 1] = transform.transforms[indices_[i]].transform.translation.y;
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 2] = transform.transforms[indices_[i]].transform.translation.z;
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 3] = transform.transforms[indices_[i]].transform.rotation.x;
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 4] = transform.transforms[indices_[i]].transform.rotation.y;
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 5] = transform.transforms[indices_[i]].transform.rotation.z;
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 6] = transform.transforms[indices_[i]].transform.rotation.w;
  }
  return true;
}

std::vector<std::string> TFRecorder::getNames() const
{
  // ROS_ASSERT_MSG(initialized_, "TFRecorder is not initialize.");
  std::vector<std::string> names;
  const int num_transforms = (int)transform_names_.size();
  for (int i = 0; i < num_transforms; ++i)
  {
    names.push_back(transform_names_[i] + std::string("_x"));
    names.push_back(transform_names_[i] + std::string("_y"));
    names.push_back(transform_names_[i] + std::string("_z"));
    names.push_back(transform_names_[i] + std::string("_qx"));
    names.push_back(transform_names_[i] + std::string("_qy"));
    names.push_back(transform_names_[i] + std::string("_qz"));
    names.push_back(transform_names_[i] + std::string("_qw"));
  }
  return names;
}

}
