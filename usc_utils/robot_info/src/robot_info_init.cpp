/*
 * robot_info.cpp
 *
 *  Created on: Dec 11, 2010
 *      Author: kalakris
 */

#include <usc_utilities/assert.h>

#include <robot_info/robot_info.h>
// #include <robot_info/trajectory_timing_generator.h>

namespace robot_info
{

void init()
{
  ROS_VERIFY(RobotInfo::initialize());
  // TrajectoryTimingGenerator::initialize();
}

}
