/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file          dynamic_movement_primitive_task.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 4, 2010

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_TASK_BASE_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_TASK_BASE_H_

// system include
#include <string>
#include <boost/shared_ptr.hpp>

namespace dmp_lib
{

/*! Parameters of a DMP that need to be save to file
 */
class DynamicMovementPrimitiveTask
{

  /*! Allow the DynamicMovementPrimitive class to access private member variables directly
   */
  friend class DynamicMovementPrimitive;

public:

  /*! Constructor
   */
  DynamicMovementPrimitiveTask() :
    object_name_("") {};

  /*! Destructor
   */
  virtual ~DynamicMovementPrimitiveTask() {};

  /*!
   * @param task
   * @return True if equal, otherwise False
   */
  bool operator==(const DynamicMovementPrimitiveTask &task) const
  {
    return (object_name_.compare(task.object_name_) == 0);
  }
  bool operator!=(const DynamicMovementPrimitiveTask &task) const
  {
    return !(*this == task);
  }

  /*!
   * @param object_name
   * @return True on success, otherwise False
   */
  bool initialize(const std::string& object_name = std::string(""))
  {
    object_name_.assign(object_name);
    return true;
  }

  /*!
   * @param object_name
   * @return True on success, otherwise False
   */
  bool get(std::string& object_name) const
  {
    object_name.assign(object_name_);
    return true;
  }

  /*! Checks whether provided object name matches the (internal) object name
   * @param object_name
   * @return True on success, otherwise False
   */
  bool isGoalObject(const std::string object_name) const
  {
    return (object_name_.compare(object_name) == 0);
  }

private:

  static const double EQUALITY_PRECISSION = 1e-6;

  /*!
   */
  std::string object_name_;

};

/*! Abbreviation for convinience
 */
typedef DynamicMovementPrimitiveTask DMPTask;
typedef boost::shared_ptr<DMPTask> DMPTaskPtr;
typedef boost::shared_ptr<DMPTask const> DMPTaskConstPtr;

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_TASK_BASE_H_ */
