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

#include <std_msgs/Empty.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>

// local includes
#include <task_recorder2_recorders/tf_recorder.h>

namespace task_recorder2_recorders
{

bool TFRecorder::readParams(ros::NodeHandle& node_handle)
{
  if(!usc_utilities::read(node_handle, "transforms", transform_names_))
  {
    ROS_WARN("Not read any transforms. Not recording them.");
    return false;
  }
  ROS_DEBUG("Starting tf recorder for transform:");
  for (int i = 0; i < (int)transform_names_.size(); ++i)
  {
    ROS_DEBUG("%i) %s", i+1, transform_names_[i].c_str());
  }
  return true;
}

bool TFRecorder::transformMsg(const task_recorder2_msgs::DataSample& msg,
                              task_recorder2_msgs::DataSample& data_sample)
{
  ROS_ASSERT(static_cast<int>(data_sample.data.size()) == getNumSignals());
  tf::StampedTransform transform;
  for (unsigned int i = 0; i < transform_names_.size(); ++i)
  {
    try
    {
      if (!tf_listener_.waitForTransform(data_sample.header.frame_id, transform_names_[i], data_sample.header.stamp,
                                         ros::Duration(0.05), ros::Duration(0.001)))
      {
        ROS_WARN("Missed transform >%s<, skipping.", transform_names_[i].c_str());
        return false;
      }
      tf_listener_.lookupTransform(data_sample.header.frame_id, transform_names_[i], data_sample.header.stamp, transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ASSERT_MSG(false, "Could not get transform from >%s< to >%s<...", data_sample.header.frame_id.c_str(), transform_names_[i].c_str());
    }
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 0] = transform.getOrigin().getX();
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 1] = transform.getOrigin().getY();
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 2] = transform.getOrigin().getZ();
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 3] = transform.getRotation().getW();
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 4] = transform.getRotation().getX();
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 5] = transform.getRotation().getY();
    data_sample.data[(i * NUM_SIGNALS_PER_TRANSFORM) + 6] = transform.getRotation().getZ();
  }
  return true;
}

std::vector<std::string> TFRecorder::getNames() const
{
  std::vector<std::string> names;
  for (int i = 0; i < (int)transform_names_.size(); ++i)
  {
    std::string transform_name = transform_names_[i];
    usc_utilities::removeLeadingSlash(transform_name);
    names.push_back(transform_name + std::string("_X"));
    names.push_back(transform_name + std::string("_Y"));
    names.push_back(transform_name + std::string("_Z"));
    names.push_back(transform_name + std::string("_QW"));
    names.push_back(transform_name + std::string("_QX"));
    names.push_back(transform_name + std::string("_QY"));
    names.push_back(transform_name + std::string("_QZ"));
  }
  return names;
}

}
