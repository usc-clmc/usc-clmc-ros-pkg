/*
 * pose_recorder.cpp
 *
 *  Created on: Jul 20, 2011
 *      Author: schorfi
 */

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_utilities/task_recorder_utilities.h>

// local includes
#include <task_recorder2/pose_recorder.h>

namespace task_recorder2
{

bool PoseRecorder::transformMsg(const geometry_msgs::PoseStamped& pose, task_recorder2_msgs::DataSample& data_sample)
{
	data_sample.header = pose.header;
	data_sample.data[POSITION_X] = pose.pose.position.x;
	data_sample.data[POSITION_Y] = pose.pose.position.y;
	data_sample.data[POSITION_Z] = pose.pose.position.z;
	data_sample.data[POSITION_QW] = pose.pose.orientation.w;
	data_sample.data[POSITION_QX] = pose.pose.orientation.x;
	data_sample.data[POSITION_QY] = pose.pose.orientation.y;
	data_sample.data[POSITION_QZ] = pose.pose.orientation.z;
	return true;
}

std::vector<std::string> PoseRecorder::getNames() const
{
	  std::vector<std::string> names;
	  names.push_back("position_x");
	  names.push_back("position_y");
	  names.push_back("position_z");
	  names.push_back("position_qw");
	  names.push_back("position_qx");
	  names.push_back("position_qy");
	  names.push_back("position_qz");
	  return names;
}

}


