/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		svm_io.h

  \author	Peter Pastor
  \date		Jul 8, 2011

 *********************************************************************/

#ifndef SVM_IO_H_
#define SVM_IO_H_

// system includes
#include <ros/ros.h>
#include <string>

#include <task_recorder2_msgs/Description.h>

// local includes
#include <task_event_detector/svm_classifier.h>

namespace task_event_detector
{

class SVMIO
{

public:

  /*! Constructor
   */
  SVMIO(ros::NodeHandle node_handle = ros::NodeHandle("/TaskEventDetector"));

  /*! Destructor
   */
  virtual ~SVMIO() {};

  /*!
   * @param description
   * @param svm_classifier
   * @return True on success, otherwise False
   */
  bool read(const task_recorder2_msgs::Description& description,
            SVMClassifierPtr& svm_classifier) const;

  /*!
   * @param description
   * @param svm_classifier
   * @return True on success, otherwise False
   */
  bool write(const task_recorder2_msgs::Description& description,
             SVMClassifierPtr& svm_classifier) const;

  /*!
   * @param descriptions
   * @return True on success, otherwise False
   * @return
   */
  bool getList(std::vector<std::string>& descriptions);

private:

  ros::NodeHandle node_handle_;
  std::string data_directory_name_;

};

}

#endif /* SVM_IO_H_ */
