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
#include <boost/shared_ptr.hpp>

#include <pr2_template_based_grasping/grasp_planning_server.h>
#include <pr2_template_based_grasping/interactive_candidate_filter.h>

using namespace std;
using namespace pr2_template_based_grasping;

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    ROS_ERROR_STREAM("You missed some arguments. The correct call is: "
        << "template_grasp_planning_server [grasp_demonstrations_path] "
        "[grasp_library_path] [negatives_path] [successes_path] ...");
    return -1;
  }

  ros::init(argc, argv, "template_grasp_planning_server");
  ros::NodeHandle n;
//  double exr = 0.0;
//  ros::param::get("~exclusion_radius", exr);
  boost::shared_ptr<const InteractiveCandidateFilter> filter;
  filter.reset(new InteractiveCandidateFilter(n));
  GraspPlanningServer planning(n, argv[1], argv[2], argv[3], argv[4], argv[5]);
  planning.attachICFilter(filter);
  ros::spin();
  std::cout << "DONE" << std::endl;
  return 0;
}
