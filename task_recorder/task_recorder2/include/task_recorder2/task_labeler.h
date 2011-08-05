/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_labeler.h

  \author	Peter Pastor
  \date		Jun 17, 2011

 *********************************************************************/

#ifndef TASK_LABELER_H_
#define TASK_LABELER_H_

// system includes
#include <string>
#include <map>
#include <ros/ros.h>
// #include <boost/thread.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/TaskRecorderSpecification.h>
#include <task_recorder2_msgs/Accumulate.h>

#include <task_recorder2_utilities/task_recorder_specification_utilities.h>

// local includes
#include <task_recorder2/task_labeler_io.h>

namespace task_recorder2
{

template<class MessageType>
  class TaskLabeler
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    TaskLabeler(ros::NodeHandle node_handle);
    /*! Destructor
     */
    virtual ~TaskLabeler() {};

    /*!
     * @param description
     * @param label_type
     * @return True on success, otherwise False
     */
    bool getLabelType(const task_recorder2_msgs::Description description,
                      int& label_type);

    /*!
     * @param descriptions
     * @param label_type
     * @return True all provided descriptions have a common label type, otherwise False
     */
    bool getLabelType(const std::vector<task_recorder2_msgs::Description>& descriptions,
                      int& label_type);

    /*!
     * @param descriptions
     * @return True all provided descriptions have a common label type, otherwise False
     */
    bool getLabelType(const std::vector<task_recorder2_msgs::Description>& descriptions)
    {
      int label_type = 0;
      return getLabelType(descriptions, label_type);
    }

    /*!
     * @param description
     * @param label
     * @return True on success, otherwise False
     */
    bool label(const task_recorder2_msgs::Description description,
               const MessageType label);

  private:

    TaskLabelerIO<task_recorder2_msgs::DataSampleLabel> labeler_io_;
    std::vector<TaskLabelerIO<task_recorder2_msgs::DataSampleLabel> > labeler_ios_resample_;
    std::vector<TaskLabelerIO<task_recorder2_msgs::DataSampleLabel> > labeler_ios_raw_;

    std::map<std::string, int> description_label_type_map_;
    std::map<std::string, int>::const_iterator description_label_type_map_iterator_;

    bool automatic_accumulation_;
    ros::ServiceClient accumulate_service_;

    // bool readDescriptionLabelTypeMap();
    bool accumulate(const task_recorder2_msgs::Description& description);

  };

template<class MessageType>
  TaskLabeler<MessageType>::TaskLabeler(ros::NodeHandle node_handle) :
    labeler_io_(node_handle)
  {
    ROS_VERIFY(labeler_io_.initialize());
    accumulate_service_ = labeler_io_.node_handle_.serviceClient<task_recorder2_msgs::Accumulate> ("/TaskAccumulator/accumulate");
    ROS_VERIFY(usc_utilities::read(labeler_io_.node_handle_, "automatic_accumulation", automatic_accumulation_));
    if (labeler_io_.write_out_resampled_data_ || labeler_io_.write_out_raw_data_)
    {
      std::vector<task_recorder2_msgs::TaskRecorderSpecification> specifications;
      ROS_VERIFY(task_recorder2_utilities::readTaskRecorderSpecification(specifications));
      if (labeler_io_.write_out_resampled_data_)
      {
        for (int i = 0; i < (int)specifications.size(); ++i)
        {
          TaskLabelerIO<task_recorder2_msgs::DataSampleLabel> labeler_io(labeler_io_.node_handle_);
          ROS_VERIFY(labeler_io.initialize(specifications[i].topic_name));
          labeler_ios_resample_.push_back(labeler_io);
        }
      }
      if (labeler_io_.write_out_raw_data_)
      {
        for (int i = 0; i < (int)specifications.size(); ++i)
        {
          TaskLabelerIO<task_recorder2_msgs::DataSampleLabel> labeler_io(labeler_io_.node_handle_);
          ROS_VERIFY(labeler_io.initialize(specifications[i].topic_name));
          labeler_ios_raw_.push_back(labeler_io);
        }
      }
    }
    ROS_VERIFY(task_recorder2_utilities::readDescriptionLabelTypeMap(labeler_io_.node_handle_, description_label_type_map_));
  }

template<class MessageType>
  bool TaskLabeler<MessageType>::getLabelType(const task_recorder2_msgs::Description description, int& label_type)
  {
    description_label_type_map_iterator_ = description_label_type_map_.find(description.description);
    if(description_label_type_map_iterator_ == description_label_type_map_.end())
    {
      ROS_ERROR("Unknown description >%s<. Cannot obtain label type.", description.description.c_str());
      return false;
    }
    label_type = description_label_type_map_iterator_->second;
    return true;
  }

template<class MessageType>
  bool TaskLabeler<MessageType>::getLabelType(const std::vector<task_recorder2_msgs::Description>& descriptions,
                                              int& label_type)
  {
    int previous_label_type = 0;
    bool selected_items_have_same_label_type = true;
    for (int i = 0; i < (int)descriptions.size() && selected_items_have_same_label_type; ++i)
    {
      ROS_DEBUG("Getting label type for >%s<.", descriptions[i].description.c_str());
      ROS_VERIFY(getLabelType(descriptions[i], label_type));
      if (i == 0)
      {
        previous_label_type = label_type;
      }
      else
      {
        if (previous_label_type != label_type)
        {
          selected_items_have_same_label_type = false;
        }
      }
    }
    return selected_items_have_same_label_type;
  }

template<class MessageType>
  bool TaskLabeler<MessageType>::label(const task_recorder2_msgs::Description description,
                                       const MessageType label)
  {
    if(labeler_io_.write_out_resampled_data_)
    {
      for (int i = 0; i < (int)labeler_ios_resample_.size(); ++i)
      {
        ROS_VERIFY(labeler_ios_resample_[i].appendResampledLabel(description, label));
      }
    }
    if(labeler_io_.write_out_raw_data_)
    {
      for (int i = 0; i < (int)labeler_ios_raw_.size(); ++i)
      {
        ROS_VERIFY(labeler_ios_raw_[i].appendRawLabel(description, label));
      }
    }
    ROS_VERIFY(labeler_io_.appendLabel(description, label));
    if(automatic_accumulation_)
    {
      ROS_VERIFY(accumulate(description));
    }
    return true;
  }

template<class MessageType>
  bool TaskLabeler<MessageType>::accumulate(const task_recorder2_msgs::Description& description)
  {
    task_recorder2_msgs::Accumulate::Request request;
    request.description = description;
    task_recorder2_msgs::Accumulate::Response response;

    bool service_online = false;
    while (ros::ok() && !service_online)
    {
      if (!accumulate_service_.waitForExistence(ros::Duration(0.5)))
      {
        ROS_WARN("Waiting for >%s< ...", accumulate_service_.getService().c_str());
      }
      else
      {
        service_online = true;
      }
    }
    ROS_VERIFY(accumulate_service_.call(request, response));
    ROS_INFO_STREAM(response.info);
    return true;
  }

}

#endif /* TASK_LABELER_H_ */
