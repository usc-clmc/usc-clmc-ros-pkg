/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_gui_node.cpp

  \author	Peter Pastor
  \date		Jun 16, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <QApplication>
#include <boost/shared_ptr.hpp>

#include <signal.h>
#include <unistd.h>

#include <robot_info/robot_info_init.h>

// local includes
#include <dynamic_movement_primitive_gui/dynamic_movement_primitive_gui.h>

using namespace dynamic_movement_primitive_gui;

void sigCatcher(int sig)
{
  if (sig == SIGINT)
  {
    puts("Caught SIGINT... shutting down ROS...");
    ros::shutdown();
    exit(0);
  }
  else if (sig == SIGTERM)
  {
    puts("Caught SIGTERM... shutting down ROS...");
    ros::shutdown();
    exit(0);
  }
}

int main(int argc,
         char *argv[])
{
  robot_info::init();
  ros::init(argc, argv, "DynamicMovementPrimitiveGui");

  (void) signal(SIGINT, sigCatcher);
  (void) signal(SIGTERM, sigCatcher);

  QApplication app(argc, argv);

  ros::NodeHandle node_handle("~");
  DynamicMovementPrimitiveGUI dynamic_movement_primitive_gui(node_handle);
  dynamic_movement_primitive_gui.show();

  ros::Duration(0.5).sleep();
  ros::AsyncSpinner async_spiner(1);
  async_spiner.start();
  return app.exec();
}
