/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   test_dmp_node.cpp

  \author Peter Pastor
  \date   Jan 12, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <robot_info/robot_info_init.h>

// local includes
#include <dmp_behaviors/test_dmp.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  robot_info::init();
  ros::NodeHandle node_handle("~");
  std::string action_name = "testDMP";
  dmp_behaviors::TestDMP test_dmp(node_handle, action_name);
  test_dmp.start();
  ros::spin();
  return 0;
}
