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
#include <QThread>
#include <boost/shared_ptr.hpp>

#include <task_recorder2_msgs/Description.h>

#include <signal.h>
#include <unistd.h>

// local includes
#include <task_label_gui/label_gui.h>

using namespace task_label_gui;

//void sigCatcher(int sig)
//{
//  if (sig == SIGINT)
//  {
//    puts("Caught SIGINT... shutting down ROS...");
//    ros::shutdown();
//    exit(0);
//  }
//  else if (sig == SIGTERM)
//  {
//    puts("Caught SIGTERM... shutting down ROS...");
//    ros::shutdown();
//    exit(0);
//  }
//}

class Application : public QThread
{
public:
  Application(QApplication* qapp, ros::NodeHandle node_handle) :
      app_(qapp)
  {
    qRegisterMetaType<std::vector<task_recorder2_msgs::Description> >("std::vector<task_recorder2_msgs::Description>");
    qRegisterMetaType<task_recorder2_msgs::Description>("task_recorder2_msgs::Description");
    label_gui_ = new LabelGui(node_handle);
  }
  void quit()
  {
    app_->quit();
  }
  void run()
  {
    ros::spin();
    quit();
  }
private:
  QApplication* app_;
  LabelGui* label_gui_;
};

int main(int argc,
         char *argv[])
{
  ros::init(argc, argv, "LabelGui");
  ros::NodeHandle node_handle("~");

//  (void) signal(SIGINT, sigCatcher);
//  (void) signal(SIGTERM, sigCatcher);

  QApplication* qapp = new QApplication(argc, argv);
  Application app(qapp, node_handle);

  app.start();
  qapp->exec();
  app.exit();
  return 0;
}
