/*
 * test_blackboard_client.cpp
 *
 *  Created on: Oct 13, 2013
 *      Author: pastor
 */




// system includes
#include <ros/ros.h>
// local includes
#include <blackboard/blackboard_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestBlackBoardClient");
  ros::NodeHandle node_handle("~");

  blackboard::BlackBoardClient right_blackboard_client;
  if(!right_blackboard_client.initialize("right"))
    return -1;
  blackboard::BlackBoardClient left_blackboard_client;
  if(!left_blackboard_client.initialize("left"))
    return -1;

  while (ros::ok())
  {
    right_blackboard_client.debug("key1", "value1");
    left_blackboard_client.debug("key1", "value1");

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    right_blackboard_client.info("key2", "value2");
    left_blackboard_client.info("key2", "value2");

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    right_blackboard_client.warn("key3", "value3");
    left_blackboard_client.warn("key3", "value3");

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    right_blackboard_client.error("key4", "value4");
    left_blackboard_client.error("key4", "value4");

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    right_blackboard_client.fatal("key5", "value5");
    left_blackboard_client.fatal("key5", "value5");
  }
  return 0;
}
