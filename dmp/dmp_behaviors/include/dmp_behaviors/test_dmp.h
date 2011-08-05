/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   test_dmp.h

  \author Peter Pastor
  \date   Jan 23, 2011

 *********************************************************************/

#ifndef TEST_DMP_H_
#define TEST_DMP_H_

// system includes
#include <string>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h>
#include <skill_library/skill_library_client.h>

// local includes
#include <dmp_behavior_actions/TestDMPAction.h>

namespace dmp_behaviors
{

class TestDMP
{

public:

  typedef actionlib::SimpleActionServer<dmp_behavior_actions::TestDMPAction> ActionServer;
  typedef ActionServer::GoalHandle GoalHandle;
  typedef dmp_behavior_actions::TestDMPResult ActionResult;

  TestDMP(ros::NodeHandle& node_handle, const std::string& action_name);
  virtual ~TestDMP() {};

  void start();
  void execute(const dmp_behavior_actions::TestDMPGoalConstPtr& goal);

private:

  ros::NodeHandle node_handle_;
  ActionServer action_server_;

  dmp_utilities::DynamicMovementPrimitiveControllerClient dmp_controller_client_;
  skill_library::SkillLibraryClient skill_library_client_;

  std::string dmp_controller_name_;

  /*!
   * @return
   */
  bool readParams();
};

}

#endif /* TEST_DMP_H_ */
