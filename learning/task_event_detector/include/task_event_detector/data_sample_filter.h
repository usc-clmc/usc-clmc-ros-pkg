/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks      ...

  \file     data_sample_filter.h

  \author   Peter Pastor
  \date     Aug 9, 2011

 *********************************************************************/

#ifndef DATA_SAMPLE_FILTER_H_
#define DATA_SAMPLE_FILTER_H_

// system includes
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>

#include <task_recorder2_msgs/DataSample.h>

// local includes
#include <task_event_detector/DataSampleFilterSpecification.h>

namespace task_event_detector
{

class DataSampleFilter
{

public:

  /*!
   */
  DataSampleFilter(ros::NodeHandle node_handle = ros::NodeHandle("/TaskEventDetector"));

  /*!
   */
  virtual ~DataSampleFilter() {};

  /*!
   * @param default_data_sample
   * @return True on success, otherwise False
   */
  bool initialize(const task_recorder2_msgs::DataSample& default_data_sample);

  /*!
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool filter(task_recorder2_msgs::DataSample& data_sample);
  bool filter(std::vector<task_recorder2_msgs::DataSample>& data_samples)
  {
    for(int i=0; i<(int)data_samples.size(); ++i)
    {
      if(!filter(data_samples[i]))
      {
        return false;
      }
    }
    return true;
  }

private:

  bool initialized_;

  /*!
   */
  std::vector<task_recorder2_msgs::DataSample> data_samples_;

  /*!
   */
  std::vector<DataSampleFilterSpecification> data_sample_filters_;
  bool readParams(ros::NodeHandle node_handle);

};

typedef boost::shared_ptr<DataSampleFilter> DataSampleFilterPtr;

}

#endif /* DATA_SAMPLE_FILTER_H_ */
