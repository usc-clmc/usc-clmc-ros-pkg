/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		policy_improvement_loop_test.cpp

  \author	Peter Pastor
  \date		Feb 21, 2011

 *********************************************************************/

// system includes
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <policy_improvement_loop/policy_improvement_loop.h>

// local includes

using namespace policy_improvement_loop;

TEST(policy_improvement_loop_test, dmp_waypoint_task)
{
    ros::NodeHandle node_handle("~");
    PolicyImprovementLoop pi_loop;
    std::string task_name = "policy_improvement_loop_test/DMPWaypointTask";
    pi_loop.initializeAndRunTaskByName(node_handle, task_name);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "policy_improvement_loop_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
