/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_dynamic_movement_primitive_gui_node.cpp

  \author	Peter Pastor
  \date		Jul 28, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <QApplication>
#include <boost/shared_ptr.hpp>

#include <signal.h>
#include <unistd.h>

#include <robot_info/robot_info_init.h>

// local includes
#include <pr2_dynamic_movement_primitive_gui/pr2_dynamic_movement_primitive_gui.h>

using namespace pr2_dynamic_movement_primitive_gui;

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
  ros::init(argc, argv, "PR2DynamicMovementPrimitiveGui");
  ros::NodeHandle node_handle("~");
  robot_info::init();

  (void) signal(SIGINT, sigCatcher);
  (void) signal(SIGTERM, sigCatcher);

  QApplication app(argc, argv);

  PR2DynamicMovementPrimitiveGUI pr2_dynamic_movement_primitive_gui(node_handle);
  pr2_dynamic_movement_primitive_gui.show();

  ros::Duration(0.5).sleep();
  ros::AsyncSpinner async_spiner(1);
  async_spiner.start();
  return app.exec();
}
