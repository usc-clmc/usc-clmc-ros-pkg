/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         template_grasp_planning_server.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <ros/ros.h>

#include <pr2_template_based_grasping/grasp_planning_server.h>

using namespace std;
using namespace pr2_template_based_grasping;

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    ROS_ERROR_STREAM("You missed some arguments. The correct call is: "
        << "template_grasp_planning_server [grasp_demonstrations_path] "
        "[grasp_library_path] [demo_filename] ...");
    return -1;
  }

  ros::init(argc, argv, "template_grasp_planning_server");
  ros::NodeHandle n;
  GraspPlanningServer planning(n, argv[1], argv[2], argv[3]);
  ros::spin();
  return 0;
}
