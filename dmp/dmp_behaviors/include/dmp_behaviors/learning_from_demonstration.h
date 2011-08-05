/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   learning_from_demonstration.h

  \author Peter Pastor
  \date   Jan 23, 2011

 *********************************************************************/

#ifndef LEARNING_FROM_DEMONSTRATION_H_
#define LEARNING_FROM_DEMONSTRATION_H_

// system includes
#include <string>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_learner_utilities.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h>

// local includes
#include <dmp_behavior_actions/LearningFromDemonstrationAction.h>

namespace dmp_behaviors
{

class LearningFromDemonstration
{

public:

  typedef actionlib::SimpleActionServer<dmp_behavior_actions::LearningFromDemonstrationAction> ActionServer;
  typedef ActionServer::GoalHandle GoalHandle;
  typedef dmp_behavior_actions::LearningFromDemonstrationResult ActionResult;

  LearningFromDemonstration(ros::NodeHandle& node_handle, const std::string& action_name);
  virtual ~LearningFromDemonstration() {};

  void start();
  void execute(const dmp_behavior_actions::LearningFromDemonstrationGoalConstPtr& goal);

private:

  ros::NodeHandle node_handle_;
  ActionServer action_server_;

  std::string demonstrations_directory_path_;
  ros::ServiceClient add_affordance_service_client_;

  dmp_utilities::DynamicMovementPrimitiveLearnerUtilities dmp_learner_utilities_;
  dmp_utilities::DynamicMovementPrimitiveControllerClient right_dmp_controller_client_;

  /*!
   * @return
   */
  bool readParams();
  std::vector<double> default_finger_start_;
  std::vector<double> default_finger_goal_;

};

}

#endif /* LEARNING_FROM_DEMONSTRATION_H_ */
