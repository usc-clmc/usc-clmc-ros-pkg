/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_monitor_gui_node.cpp

  \author	Peter Pastor
  \date		Jun 16, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <vector>
#include <QApplication>
#include <boost/shared_ptr.hpp>

#include <task_event_detector/shogun_init.h>
#include <task_recorder2_msgs/Description.h>

#include <signal.h>
#include <unistd.h>

// local includes
#include <task_monitor_gui/task_monitor_gui.h>

using namespace task_monitor_gui;

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
  ros::init(argc, argv, "TaskMonitorGui");
  ros::NodeHandle node_handle("~");
  task_event_detector::init();

  (void) signal(SIGINT, sigCatcher);
  (void) signal(SIGTERM, sigCatcher);

  QApplication app(argc, argv);
  qRegisterMetaType<std::vector<task_recorder2_msgs::Description> >("std::vector<task_recorder2_msgs::Description>");
  qRegisterMetaType<task_recorder2_msgs::Description>("task_recorder2_msgs::Description");

  TaskMonitorGui task_monitor_gui(node_handle);
  task_monitor_gui.show();

  ros::Duration(0.5).sleep();
  ros::AsyncSpinner async_spiner(1);
  async_spiner.start();

  int return_code = app.exec();
  task_event_detector::exit();
  return return_code;
}
