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

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_TASK_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_TASK_H_

// system include
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>

#include <dmp_lib/logger.h>

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
    object_name_(""), endeffector_id_(-1)
  {
    palm_to_tool_ = Eigen::VectorXd::Zero(3 + 4);
    object_to_tool_ = Eigen::VectorXd::Zero(3 + 4);
  };

  /*! Destructor
   */
  virtual ~DynamicMovementPrimitiveTask() {};

  /*!
   * @param task
   * @return True if equal, otherwise False
   */
  bool operator==(const DynamicMovementPrimitiveTask &task) const
  {
    // ignore transforms for now...
    return (object_name_.compare(task.object_name_) == 0
        || endeffector_id_ == task.endeffector_id_);
  }
  bool operator!=(const DynamicMovementPrimitiveTask &task) const
  {
    return !(*this == task);
  }

  /*!
   * @param object_name
   * @param palm_to_tool_pose
   * @param object_to_tool_pose
   * @param endeffector_id
  * @return True on success, otherwise False
   */
  bool initialize(const std::string& object_name,
                  const int& endeffector_id,
                  const Eigen::VectorXd palm_to_tool_pose = Eigen::VectorXd::Zero(3 + 4),
                  const Eigen::VectorXd object_to_tool_pose = Eigen::VectorXd::Zero(3 + 4))
  {
    setObjectName(object_name);
    setPalmToToolPose(palm_to_tool_pose);
    setObjectToToolPose(object_to_tool_pose);
    setEndeffectorId(endeffector_id);
    return true;
  }

  /*!
   * @param object_name
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  const std::string& getObjectName() const
  {
    return object_name_;
  }
  /*!
   * @param object_name
   */
  void setObjectName(const std::string& object_name)
  {
    object_name_.assign(object_name);
  }

  /*! Checks whether provided object name matches the (internal) object name
   * @param object_name
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool isGoalObject(const std::string object_name) const
  {
    return (object_name_.compare(object_name) == 0);
  }

  /*!
   * @param palm_to_tool_pose
   */
  void setPalmToToolPose(const Eigen::VectorXd& palm_to_tool_pose)
  {
    assert(palm_to_tool_pose.rows() == 3 + 4);
    palm_to_tool_ = palm_to_tool_pose;
  }
  /*!
   * @return the pose of the tool in palm frame
   * REAL-TIME REQUIREMENTS
   */
  const Eigen::VectorXd& getPalmToToolPose() const
  {
    return palm_to_tool_;
  }

  /*!
   * @param palm_to_tool_pose
   */
  void setObjectToToolPose(const Eigen::VectorXd& object_to_tool_pose)
  {
    assert(object_to_tool_pose.rows() == 3 + 4);
    object_to_tool_ = object_to_tool_pose;
  }
  /*!
   * @return the pose of the tool in object frame
   * REAL-TIME REQUIREMENTS
   */
  const Eigen::VectorXd& getObjectToToolPose() const
  {
    return object_to_tool_;
  }

  /*!
   * @param endeffector_id
   */
  void setEndeffectorId(const int endeffector_id)
  {
    endeffector_id_ = endeffector_id;
  }
  /*! Note: this variable is set when sending the message to the dmp controller
   * @return endeffector id
   * REAL-TIME REQUIREMENTS
   */
  const int& getEndeffectorId() const
  {
    if (endeffector_id_ < 0)
      Logger::logPrintf("Endeffector id is not set >%i<. (Real-time violations).", Logger::ERROR, endeffector_id_);
    return endeffector_id_;
  }

private:

  static const double EQUALITY_PRECISSION = 1e-6;

  /*!
   */
  std::string object_name_;

  /*!
   */
  Eigen::VectorXd palm_to_tool_;
  Eigen::VectorXd object_to_tool_;

  /*!
   */
  int endeffector_id_;

};

/*! Abbreviation for convenience
 */
typedef DynamicMovementPrimitiveTask DMPTask;
typedef boost::shared_ptr<DMPTask> DMPTaskPtr;
typedef boost::shared_ptr<DMPTask const> DMPTaskConstPtr;

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_TASK_BASE_H_ */
