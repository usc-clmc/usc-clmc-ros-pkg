/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		skill_library_node.cpp

  \author	Peter Pastor
  \date		Jan 23, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
// #include <pr2_controller_utilities/pr2_controller_utilities.h>

// local includes
#include <skill_library/skill_library.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "/SkillLibrary");
  // pr2_controller_utilities::init();
  skill_library::SkillLibrary skill_library;
  if(!skill_library.initialize())
  {
    ROS_ERROR("Could not initialize skill library.");
    return -1;
  }
  return skill_library.run();
}
