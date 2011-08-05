/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    test_dmp_joint_trajectory_controller.cpp

 \author  Peter Pastor
 \date    Jan 10, 2011

 *********************************************************************/

// system includes
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

using namespace std;

static const string package_name = "dynamic_movement_primitive_controller_test";

int main(int argc, char** argv)
{
  ros::init(argc, argv, package_name);
  ros::NodeHandle node_handle("~");

  std::string dmp_bag_file_name;
  ROS_VERIFY(usc_utilities::read(node_handle, "dmp_bag_file", dmp_bag_file_name));
  string abs_bag_file_name_dmp = ros::package::getPath(package_name) + "/data/" + dmp_bag_file_name;

  std::string topic;
  ROS_VERIFY(usc_utilities::read(node_handle, "topic", topic));

  std::string dmp_version;
  ROS_VERIFY(usc_utilities::read(node_handle, "dmp_version", dmp_version));
  if(dmp_version.compare("icra2009") == 0)
  {
    dmp_lib::ICRA2009DMPPtr icra2009_dmp;
    ROS_VERIFY(dmp::ICRA2009DynamicMovementPrimitive::readFromDisc(icra2009_dmp, abs_bag_file_name_dmp));
		ROS_VERIFY(icra2009_dmp->setup());

    dmp::ICRA2009DynamicMovementPrimitive::DMPMsg icra2009_dmp_msg;
    ROS_VERIFY(dmp::ICRA2009DynamicMovementPrimitive::writeToMessage(icra2009_dmp, icra2009_dmp_msg));
    ROS_INFO("Advertising... >%s<.", topic.c_str());
    ros::Publisher publisher = node_handle.advertise<dmp::ICRA2009DynamicMovementPrimitive::DMPMsg>(topic, 10);
    ros::Duration(1.0).sleep();
    ROS_INFO("Publishing... >%s< DMP.", dmp_version.c_str());
    publisher.publish(icra2009_dmp_msg);
    ROS_INFO("Publishing... >%s< DMP ...done !", dmp_version.c_str());
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }
  else if(dmp_version.compare("nc2010") == 0)
  {
    dmp_lib::NC2010DMPPtr nc2010_dmp;
    ROS_VERIFY(dmp::NC2010DynamicMovementPrimitive::readFromDisc(nc2010_dmp, abs_bag_file_name_dmp));
		ROS_VERIFY(nc2010_dmp->setup());
    dmp::NC2010DynamicMovementPrimitive::DMPMsg nc2010_dmp_msg;
    ROS_VERIFY(dmp::NC2010DynamicMovementPrimitive::writeToMessage(nc2010_dmp, nc2010_dmp_msg));
    ROS_INFO("Advertising... >%s<.", topic.c_str());
    ros::Publisher publisher = node_handle.advertise<dmp::NC2010DynamicMovementPrimitive::DMPMsg>(topic, 10);
    ros::Duration(1.0).sleep();
    ROS_INFO("Publishing... >%s< DMP.", dmp_version.c_str());
    publisher.publish(nc2010_dmp_msg);
    ROS_INFO("Publishing... >%s< DMP ...done !", dmp_version.c_str());
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }
  else
  {
    ROS_ERROR("Invalid DMP version >%s< read from param server in namespace >%s<.", dmp_version.c_str(), node_handle.getNamespace().c_str());
    return -1;
  }
  return 0;
}
