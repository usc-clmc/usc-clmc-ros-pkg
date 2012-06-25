/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks      ...
 
  \file     dynamic_movement_primitive_test.cpp

  \author   Peter Pastor, Mrinal Kalakrishnan
  \date     Dec 8, 2010

 *********************************************************************/

// system includes

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>

// local includes
#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/../../dmpLib/test/test.h>

using namespace dmp;

static std::string abs_bag_file_name_node_handle = "/tmp/test_dmp_from_node_handle.bag";
static std::string abs_bag_file_name_message = "/tmp/test_dmp_from_message.bag";

TEST(dmp_tests, dmp_lib_test)
{
  // TODO: fix this
  // std::string package_path = ros::package::getPath("dynamic_movement_primitive");
  // EXPECT_TRUE(test_dmp::Test::test(package_path + "/dmpLib/test/", false));
  EXPECT_TRUE(test_dmp::Test::test("/home/arm_user/ARM/usc-clmc-ros-pkg/dmp/dynamic_movement_primitive/dmpLib/test/", false));
}

TEST(dmp_tests, initFromNodeHandle)
{
  dmp_lib::ICRA2009DMPPtr dmp_from_node_handle;
  ros::NodeHandle node_handle("~/joint_space_dmp");

  ROS_INFO("Checking for %s.", node_handle.getNamespace().c_str());

  std::vector<std::string> robot_part_names;
  robot_part_names.push_back("RIGHT_ARM");

  bool result = ICRA2009DynamicMovementPrimitive::initFromNodeHandle(dmp_from_node_handle, robot_part_names, node_handle);
  EXPECT_TRUE(result);

  result = ICRA2009DynamicMovementPrimitive::writeToDisc(dmp_from_node_handle, abs_bag_file_name_node_handle);
  EXPECT_TRUE(result);

  dmp_lib::ICRA2009DMPPtr dmp_from_node_handle_copy;
  result = ICRA2009DynamicMovementPrimitive::readFromDisc(dmp_from_node_handle_copy, abs_bag_file_name_node_handle);
  EXPECT_TRUE(result);

  dmp_lib::Time initial_time, initial_time_copy;
  double teaching_duration, teaching_duration_copy;
  double execution_duration, execution_duration_copy;
  double cutoff, cutoff_copy;
  int type, type_copy;
  int id, id_copy;
  EXPECT_TRUE(dmp_from_node_handle->getParameters()->get(initial_time, teaching_duration, execution_duration, cutoff, type, id));
  EXPECT_TRUE(dmp_from_node_handle_copy->getParameters()->get(initial_time_copy, teaching_duration_copy, execution_duration_copy, cutoff_copy, type_copy, id_copy));
  EXPECT_TRUE(initial_time == initial_time_copy);
  EXPECT_TRUE(execution_duration == execution_duration_copy);
  EXPECT_TRUE(teaching_duration == teaching_duration_copy);
  EXPECT_TRUE(cutoff == cutoff_copy);
  EXPECT_TRUE(type == type_copy);
  EXPECT_TRUE(id == id_copy);

  EXPECT_TRUE(*dmp_from_node_handle == *dmp_from_node_handle);
  EXPECT_TRUE(*dmp_from_node_handle == *dmp_from_node_handle_copy);
  EXPECT_FALSE(*dmp_from_node_handle != *dmp_from_node_handle_copy);

  ICRA2009DMPMsg icra2009_dmp_msg;
  result = ICRA2009DynamicMovementPrimitive::writeToMessage(dmp_from_node_handle_copy, icra2009_dmp_msg);
  EXPECT_TRUE(result);

  dmp_lib::ICRA2009DMPPtr dmp_init_from_message;
  result = ICRA2009DynamicMovementPrimitive::createFromMessage(dmp_init_from_message, icra2009_dmp_msg);
  EXPECT_TRUE(result);

  EXPECT_TRUE(dmp_init_from_message->getParameters()->get(initial_time, teaching_duration, execution_duration, cutoff, type, id));
  EXPECT_TRUE(initial_time == initial_time_copy);
  EXPECT_TRUE(execution_duration == execution_duration_copy);
  EXPECT_TRUE(teaching_duration == teaching_duration_copy);
  EXPECT_TRUE(cutoff == cutoff_copy);

  result = ICRA2009DynamicMovementPrimitive::writeToDisc(dmp_init_from_message, abs_bag_file_name_message);
  EXPECT_TRUE(result);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dmp_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
