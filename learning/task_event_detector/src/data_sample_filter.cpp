/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks      ...

  \file     data_sample_filter.cpp

  \author   Peter Pastor
  \date     Aug 9, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_utilities/data_sample_utilities.h>

// local includes
#include <task_event_detector/data_sample_filter.h>

namespace task_event_detector
{

DataSampleFilter::DataSampleFilter(ros::NodeHandle node_handle) :
  initialized_(false)
{
  ROS_VERIFY(readParams(node_handle));
}

bool DataSampleFilter::initialize(const task_recorder2_msgs::DataSample& default_data_sample)
{
  for (int i = 0; i < (int)data_sample_filters_.size(); ++i)
  {
    ROS_VERIFY(task_recorder2_utilities::getIndices(default_data_sample.names, data_sample_filters_[i].input, data_sample_filters_[i].indices));
  }
  return (initialized_ = true);
}

bool DataSampleFilter::filter(task_recorder2_msgs::DataSample& data_sample)
{
  ROS_ASSERT(initialized_);
  const int DATA_SAMPLE_SIZE = data_sample.data.size();
  ROS_VERIFY_MSG(DATA_SAMPLE_SIZE == (int)data_sample.names.size(),
                 "Data sample is inconsistent. This should never happen.");

  ROS_DEBUG("Computing >%i< new outputs.", (int)data_sample_filters_.size());
  for (int i = 0; i < (int)data_sample_filters_.size(); ++i)
  {
    double value = 0;
    const int NUM_INDICES = (int)data_sample_filters_[i].indices.size();
    ROS_VERIFY_MSG(NUM_INDICES > 0, "Number of indices >%i< is incorrect.", NUM_INDICES);

    if(data_sample_filters_[i].method == DataSampleFilterSpecification::AVG_METHOD)
    {
      // compute average
      for (int j = 0; j < NUM_INDICES; ++j)
      {
        value += data_sample.data[data_sample_filters_[i].indices[j]];
      }
      value /= (double)NUM_INDICES;
    }
    else if(data_sample_filters_[i].method == DataSampleFilterSpecification::MAX_METHOD)
    {
      // compute maximum
      value = data_sample.data[data_sample_filters_[i].indices[0]];
      for (int j = 0; j < NUM_INDICES; ++j)
      {
        if(data_sample.data[data_sample_filters_[i].indices[j]] > value)
        {
          value = data_sample.data[data_sample_filters_[i].indices[j]];
        }
      }
    }
    else
    {
      ROS_ERROR("Unknown filter ethod >%i<. Not filtering.", data_sample_filters_[i].method);
    }

    // TODO: think about how to avoid memory allocation
    data_sample.names.push_back(data_sample_filters_[i].output);
    data_sample.data.push_back(value);
  }
  return true;
}

bool DataSampleFilter::readParams(ros::NodeHandle node_handle)
{
  data_sample_filters_.clear();
  // read the list of description-label_type mapping from the param server
  XmlRpc::XmlRpcValue filters;
  if (!node_handle.getParam("data_sample_filters", filters))
  {
    ROS_WARN("Couldn't find parameter >%s/data_sample_filters<.", node_handle.getNamespace().c_str());
    ROS_WARN("Not using data sample filters.");
    return true;
  }
  if (filters.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR(">%s/data_sample_filters must be a struct and not of type >%i<.",
              node_handle.getNamespace().c_str(), (int)filters.getType());
    return false;
  }

  for (int i = 0; i < filters.size(); ++i)
  {
    DataSampleFilterSpecification specification;
    if (!filters[i].hasMember(DataSampleFilterSpecification::INPUT))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", DataSampleFilterSpecification::INPUT.c_str());
      return false;
    }
    XmlRpc::XmlRpcValue input_list = filters[i][DataSampleFilterSpecification::INPUT];
    if (input_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Input in namespace >%s< must be an array.", node_handle.getNamespace().c_str());
      return false;
    }
    for (int j = 0; j < (int)input_list.size(); ++j)
    {
      std::string input_name = input_list[j];
      specification.input.push_back(input_name);
    }

    if (!filters[i].hasMember(DataSampleFilterSpecification::OUTPUT))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", DataSampleFilterSpecification::OUTPUT.c_str());
      return false;
    }
    std::string output_name = filters[i][DataSampleFilterSpecification::OUTPUT];
    specification.output = output_name;

    if (!filters[i].hasMember(DataSampleFilterSpecification::METHOD))
    {
      ROS_ERROR("Description-LabelType map must have a field \"%s\".", DataSampleFilterSpecification::METHOD.c_str());
      return false;
    }
    std::string method =  filters[i][DataSampleFilterSpecification::METHOD];
    if(method == DataSampleFilterSpecification::COMPUTE_AVERAGE)
    {
      specification.method = DataSampleFilterSpecification::AVG_METHOD;
    }
    else if(method == DataSampleFilterSpecification::COMPUTE_MAXIMUM)
    {
      specification.method = DataSampleFilterSpecification::MAX_METHOD;
    }
    else
    {
      ROS_ERROR("Invalid filter method >%s<.", method.c_str());
      return false;
    }

    data_sample_filters_.push_back(specification);
  }

  return true;
}

}
