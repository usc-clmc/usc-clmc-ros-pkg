/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		dynamic_movement_primitive.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Dec 6, 2010

 *********************************************************************/

// system includes
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/time.h>
#include <dynamic_movement_primitive/TypeMsg.h>

namespace dmp
{

bool DynamicMovementPrimitive::initFromNodeHandle(dmp_lib::DMPPtr dmp,
                                                  ros::NodeHandle& node_handle)
{
  ROS_DEBUG("Initializing DMP from node handle.");
  Time initial_time;
  ros::NodeHandle initial_time_node_handle(node_handle, "initial_time");
  ROS_VERIFY(initial_time.initFromNodeHandle(initial_time_node_handle));

  double execution_duration = 0;
  ROS_VERIFY(usc_utilities::read(node_handle, "execution_duration", execution_duration));
  double teaching_duration = 0;
  ROS_VERIFY(usc_utilities::read(node_handle, "teaching_duration", teaching_duration));

  double cutoff = 0;
  ROS_VERIFY(usc_utilities::read(node_handle, "cutoff", cutoff));

  int type = 0;
  ROS_VERIFY(usc_utilities::read(node_handle, "type", type));
  if (type < 0 || type > dynamic_movement_primitive::TypeMsg::NUM_TYPES)
  {
    ROS_ERROR("Invalid DMP type >%i< read from param server.", type);
    return false;
  }

  // initialize dmp parameters
  if (!dmp->getParameters()->initialize(initial_time, teaching_duration, execution_duration, cutoff, type))
  {
      ROS_ERROR("Could not initialize DMP parameters from node handle.");
      return false;
  }
  return true;
}

bool DynamicMovementPrimitive::initFromMessage(dmp_lib::DMPPtr dmp,
                                               const DMPMsg& dmp_msg)
{
  ROS_DEBUG("Initializing DMP from message.");

  // initialize dmp parameters
  if (!dmp->getParameters()->initialize(dmp_lib::Time(dmp_msg.parameters.initial_time.delta_t,
                                                      dmp_msg.parameters.initial_time.tau),
                                        dmp_msg.parameters.teaching_duration,
                                        dmp_msg.parameters.execution_duration,
                                        dmp_msg.parameters.cutoff,
                                        dmp_msg.parameters.type,
                                        dmp_msg.parameters.id))
  {
      ROS_ERROR("Could not initialize DMP parameters from message.");
      return false;
  }

  // set dmp state
  if(!dmp->getState()->initialize(dmp_msg.state.is_learned,
                                  dmp_msg.state.is_setup,
                                  dmp_msg.state.is_start_set,
                                  dmp_lib::Time(dmp_msg.state.current_time.delta_t,
                                                dmp_msg.state.current_time.tau),
                                  dmp_msg.state.num_training_samples,
                                  dmp_msg.state.num_generated_samples,
                                  dmp_msg.state.seq))
  {
      ROS_ERROR("Could not initialize DMP state from message.");
      return false;
  }
  return true;
}

bool DynamicMovementPrimitive::writeToMessage(dmp_lib::DMPConstPtr dmp,
                                              DMPMsg& dmp_msg)
{
  dmp_lib::DMPParamConstPtr parameters;
  dmp_lib::DMPStateConstPtr state;
  ROS_VERIFY(dmp->get(parameters, state));

  // set dmp parameters
  Time initial_time;
  double teaching_duration = 0;
  double execution_duration = 0;
  double cutoff = 0;
  int type = 0;
  int id = 0;
  ROS_VERIFY(parameters->get(initial_time, teaching_duration, execution_duration, cutoff, type, id));
  dmp_msg.parameters.initial_time = initial_time.toMessage();
  dmp_msg.parameters.teaching_duration = teaching_duration;
  dmp_msg.parameters.execution_duration = execution_duration;
  dmp_msg.parameters.cutoff = cutoff;
  dmp_msg.parameters.type = type;
  dmp_msg.parameters.id = id;

  // set dmp state
  bool is_learned = false;
  bool is_setup = false;
  bool is_start_set = false;
  Time current_time;
  int num_training_samples = 0;
  int num_generated_samples = 0;
  int seq = 0;
  ROS_VERIFY(state->get(is_learned, is_setup, is_start_set, current_time, num_training_samples, num_generated_samples, seq));
  dmp_msg.state.is_learned = is_learned;
  dmp_msg.state.is_setup = is_setup;
  dmp_msg.state.is_start_set = is_start_set;
  dmp_msg.state.current_time = current_time.toMessage();
  dmp_msg.state.num_training_samples = num_training_samples;
  dmp_msg.state.num_generated_samples = num_generated_samples;
  dmp_msg.state.seq = seq;
  return true;
}

}
