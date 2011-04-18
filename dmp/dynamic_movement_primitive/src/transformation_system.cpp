/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		transformation_system.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 7, 2010

 *********************************************************************/

// system includes
#include <stdio.h>
#include <locally_weighted_regression/locally_weighted_regression.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <dynamic_movement_primitive/transformation_system.h>
#include <dynamic_movement_primitive/state.h>

using namespace std;

namespace dmp
{

bool TransformationSystem::initFromNodeHandle(dmp_lib::TSParamPtr parameters,
                                              XmlRpc::XmlRpcValue& ts_xml,
                                              ros::NodeHandle& node_handle)
{
  double cutoff = 0;
  if(!usc_utilities::read(node_handle, "cutoff", cutoff))
  {
    ROS_ERROR("Could not read transformation system parameter >cutoff< from param server in namespace >%s<.", node_handle.getNamespace().c_str());
    return false;
  }

  std::string name;
  if (!usc_utilities::getParam(ts_xml, "name", name))
  {
    ROS_ERROR("Could not read transformation system parameter >name< from param server in namespace >%s<.", node_handle.getNamespace().c_str());
    return false;
  }

  // create lwr model and initialize lwr model from node handle
  lwr::LWRPtr lwr_model(new lwr::LocallyWeightedRegression());
  if (!lwr_model->initFromNodeHandle(node_handle, cutoff))
  {
    return false;
  }

  // initialize transformation system parameters from message
  if (!parameters->initialize(lwr_model->getModel(), name))
  {
    return false;
  }
  return true;
}

bool TransformationSystem::initFromMessage(dmp_lib::TSParamPtr parameters,
                                           dmp_lib::TSStatePtr state,
                                           const TSParamMsg& ts_param_msg,
                                           const TSStateMsg& ts_state_msg)
{
  // create lwr model and initialize lwr model from node handle
  lwr::LWRPtr lwr_model(new lwr::LocallyWeightedRegression());
  if (!lwr_model->initFromMessage(ts_param_msg.lwr_model))
  {
    return false;
  }

  // initialize transformation system parameters from message
  if (!parameters->initialize(lwr_model->getModel(), ts_param_msg.name, ts_param_msg.initial_start, ts_param_msg.initial_goal))
  {
    return false;
  }

  // initialize transformation system state from message
  State internal(ts_state_msg.internal);
  State target(ts_state_msg.target);
  State current(ts_state_msg.current);
  if (!state->set(internal.getState(), target.getState(), current.getState(), ts_state_msg.start, ts_state_msg.goal, ts_state_msg.f, ts_state_msg.ft/*,
   ts_state_msg.function_input, ts_state_msg.function_target*/))
  {
    return false;
  }
  return true;
}

//bool TransformationSystem::initFromMessage(dmp_lib::TSPtr transformation_system,
//                                           const TSMsg& ts_msg)
//{
//  for (int i = 0; i < (int)ts_msg.parameters.size(); ++i)
//  {
//    // create lwr model and initialize lwr model from node handle
//    lwr::LWRPtr lwr_model(new lwr::LocallyWeightedRegression());
//    if (!lwr_model->initFromMessage(ts_msg.parameters[i].lwr_model))
//    {
//      return false;
//    }
//
//    // initialize transformation system parameters from message
//    if (!transformation_system->getParameters()[i]->initialize(lwr_model->getModel(), ts_msg.parameters[i].name, ts_msg.parameters[i].initial_start,
//                                                               ts_msg.parameters[i].initial_goal))
//    {
//      return false;
//    }
//
//    // initialize transformation system state from message
//    State internal(ts_msg.states[i].internal);
//    State target(ts_msg.states[i].target);
//    State current(ts_msg.states[i].current);
//    if (!transformation_system->getStates()[i]->set(internal.getState(), target.getState(), current.getState(),
//                                                   ts_msg.states[i].start, ts_msg.states[i].goal,
//                                                   ts_msg.states[i].f, ts_msg.states[i].ft/*,
//     ts_msg.states[i].function_input, ts_msg.states[i].function_target*/))
//    {
//      return false;
//    }
//  }
//  return true;
//}


bool TransformationSystem::writeToMessage(const dmp_lib::TSConstPtr transformation_system,
                                          TSMsg& ts_msg)
{
  ROS_DEBUG("Writing transformation system to message.");
  vector<dmp_lib::TSParamConstPtr> parameters;
  vector<dmp_lib::TSStateConstPtr> states;

  dmp_lib::TransformationSystem::IntegrationMethod integration_method;
  if(!transformation_system->get(parameters, states, integration_method))
  {
    return false;
  }
  ts_msg.integration_method = static_cast<int>(integration_method);

  ts_msg.parameters.resize(parameters.size());
  for (int i = 0; i < (int)parameters.size(); ++i)
  {
    // set parameters
    lwr_lib::LWRConstPtr lwr_model;
    if(!parameters[i]->get(lwr_model, ts_msg.parameters[i].name, ts_msg.parameters[i].initial_start, ts_msg.parameters[i].initial_goal))
    {
      return false;
    }
    ROS_VERIFY(lwr::writeToMessage(lwr_model, ts_msg.parameters[i].lwr_model));
  }

  ts_msg.states.resize(states.size());
  for (int i = 0; i < (int)states.size(); ++i)
  {
    // set state
    dmp_lib::State internal, target, current;
    if(!states[i]->get(internal, target, current,
                          ts_msg.states[i].start, ts_msg.states[i].goal,
                          ts_msg.states[i].f, ts_msg.states[i].ft/*,
                          ts_msg.states[i].function_input, ts_msg.states[i].function_target*/))
    {
      return false;
    }
    internal.get(ts_msg.states[i].internal.x, ts_msg.states[i].internal.xd, ts_msg.states[i].internal.xdd);
    target.get(ts_msg.states[i].target.x, ts_msg.states[i].target.xd, ts_msg.states[i].target.xdd);
    current.get(ts_msg.states[i].current.x, ts_msg.states[i].current.xd, ts_msg.states[i].current.xdd);
  }

  ROS_DEBUG("Done writing transformation system to message.");
  return true;
}

}
