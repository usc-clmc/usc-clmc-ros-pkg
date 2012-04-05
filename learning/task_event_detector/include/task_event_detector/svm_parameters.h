/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		svm_parameters.h

  \author	Peter Pastor
  \date		Jul 21, 2011

 *********************************************************************/

#ifndef SVM_PARAMETERS_H_
#define SVM_PARAMETERS_H_

// system includes
#include <string>
#include <vector>
#include <ros/ros.h>

// local includes
#include <task_event_detector/SVMParametersMsg.h>
#include <task_event_detector/GridSearchParametersMsg.h>

namespace task_event_detector
{

class SVMParameters
{

  friend class SVMClassifier;

public:

  /*! Constructor
   */
  SVMParameters();

  /*! Destructor
   */
  virtual ~SVMParameters() {};

  /*!
   * @param directory_name
   * @return True on success, otherwise False
   */
  bool load(const std::string& directory_name);

  /*!
   * @param directory_name
   * @return True on success, otherwise False
   */
  bool save(const std::string& directory_name) const;

  /*!
   * @param node_handle
   * @return True on success, otherwise False
   */
  bool read(ros::NodeHandle node_handle);

  /*!
   * @param grid_search_parameters
   * @return True on success, otherwise False
   */
  bool set(task_event_detector::SVMParametersMsg& parameters);

  /*!
   * @return
   */
  task_event_detector::SVMParametersMsg get() const;

private:

  bool initialized_;
  void setDefault();
  SVMParametersMsg msg_;

};

}


#endif /* SVM_PARAMETERS_H_ */
