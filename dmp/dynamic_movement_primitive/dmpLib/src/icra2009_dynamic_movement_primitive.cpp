/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_dynamic_movement_primitive.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 23, 2010

 *********************************************************************/

// system includes
#include <stdio.h>

// local includes
#include <dmp_lib/icra2009_dynamic_movement_primitive.h>
#include <dmp_lib/icra2009_transformation_system.h>
#include <dmp_lib/icra2009_canonical_system.h>
#include <dmp_lib/logger.h>
#include <dmp_lib/utilities.h>

using namespace std;

namespace dmp_lib
{

ICRA2009DynamicMovementPrimitive& ICRA2009DynamicMovementPrimitive::operator=(const ICRA2009DynamicMovementPrimitive& icra2009dmp)
{
  Logger::logPrintf("ICRA2009DynamicMovementPrimitive assignment.", Logger::DEBUG);

  // first assign all member variables
  assert(Utilities<ICRA2009DMPParam>::assign(parameters_, icra2009dmp.parameters_));
  assert(Utilities<ICRA2009DMPState>::assign(state_, icra2009dmp.state_));
  assert(Utilities<ICRA2009TS>::assign(transformation_systems_, icra2009dmp.transformation_systems_));
  assert(Utilities<ICRA2009CS>::assign(canonical_system_, icra2009dmp.canonical_system_));

  // then assign base class variables
  DynamicMovementPrimitive::parameters_ = parameters_;
  DynamicMovementPrimitive::state_ = state_;
  DynamicMovementPrimitive::transformation_systems_.clear();
  for (int i = 0; i < (int)transformation_systems_.size(); ++i)
  {
    DynamicMovementPrimitive::transformation_systems_.push_back(transformation_systems_[i]);
  }
  DynamicMovementPrimitive::canonical_system_ = canonical_system_;

  indices_ = icra2009dmp.indices_;
  initialized_ = icra2009dmp.initialized_;
  return *this;
}

bool ICRA2009DynamicMovementPrimitive::initialize(ICRA2009DMPParamPtr& parameters,
                                                  ICRA2009DMPStatePtr& state,
                                                  vector<ICRA2009TSPtr>& transformation_systems,
                                                  ICRA2009CSPtr& canonical_system)
{
  // Logger::logPrintf(initialized_, "DMP already initialized. Re-initializing it...", Logger::WARN);

  // first set all member variables
  assert(Utilities<ICRA2009DMPParam>::assign(parameters_, parameters));
  assert(Utilities<ICRA2009DMPState>::assign(state_, state));
  assert(Utilities<ICRA2009TS>::assign(transformation_systems_, transformation_systems));
  assert(Utilities<ICRA2009CS>::assign(canonical_system_, canonical_system));

  // then initialize base class variables
  vector<TSPtr> base_transformation_systems;
  for (vector<ICRA2009TSPtr>::const_iterator vi = transformation_systems_.begin(); vi != transformation_systems_.end(); ++vi)
  {
    base_transformation_systems.push_back(*vi);
  }
  return DynamicMovementPrimitive::initialize(parameters_, state_, base_transformation_systems, canonical_system_);
}

bool ICRA2009DynamicMovementPrimitive::add(const ICRA2009DynamicMovementPrimitive& icra2009_dmp, bool check_for_compatibiliy)
{
  if(check_for_compatibiliy && !isCompatible(icra2009_dmp))
  {
    return false;
  }
  transformation_systems_.insert(transformation_systems_.end(), icra2009_dmp.transformation_systems_.begin(), icra2009_dmp.transformation_systems_.end());
  if(!setupIndices())
  {
    return false;
  }
  return parameters_->changeType(*icra2009_dmp.parameters_);
}

bool ICRA2009DynamicMovementPrimitive::get(ICRA2009DMPParamConstPtr& parameters,
                                           ICRA2009DMPStateConstPtr& state,
                                           vector<ICRA2009TSConstPtr>& transformation_systems,
                                           ICRA2009CSConstPtr& canonical_system) const
{
  if(!initialized_)
  {
    Logger::logPrintf("ICRA2009 DMP is not initialized, cannot return parameters.", Logger::ERROR);
    return false;
  }
  parameters = parameters_;
  state = state_;
  transformation_systems.clear();
  for (int i = 0; i < (int)transformation_systems_.size(); ++i)
  {
    transformation_systems.push_back(transformation_systems_[i]);
  }
  canonical_system = canonical_system_;
  return true;
}

bool ICRA2009DynamicMovementPrimitive::initialize(const vector<string>& variable_names, lwr_lib::LWRParamPtr lwr_parameters, const double k_gain, const double d_gain)
{
  assert(!variable_names.empty());
  assert(lwr_parameters->isInitialized());

  vector<TSPtr> icra2009_transformation_systems;
  for (int i = 0; i < (int)variable_names.size(); ++i)
  {
    ICRA2009TSParamPtr icra2009_transformation_system_parameters(new ICRA2009TransformationSystemParameters());

    lwr_lib::LWRPtr lwr_model(new lwr_lib::LWR());
    if (!lwr_model->initialize(lwr_parameters))
    {
      Logger::logPrintf("Could not initialize LWR model with provide parameters.", Logger::ERROR);
      return false;
    }

    if (!icra2009_transformation_system_parameters->initialize(lwr_model, variable_names[i], k_gain, d_gain))
    {
      Logger::logPrintf("Could not initialize transformation system parameters.", Logger::ERROR);
      return false;
    }

    // TODO: fix integration method !!
    ICRA2009TSStatePtr icra2009_transformation_system_state(new ICRA2009TransformationSystemState());
    ICRA2009TSPtr icra2009_transformation_system(new ICRA2009TransformationSystem());
    if (!icra2009_transformation_system->initialize(icra2009_transformation_system_parameters,
                                                    icra2009_transformation_system_state,
                                                    TransformationSystem::NORMAL))
    {
      Logger::logPrintf("Could not initialize transformation system.", Logger::ERROR);
      return false;
    }
    icra2009_transformation_systems.push_back(icra2009_transformation_system);
  }

  ICRA2009CSParamPtr icra2009_canonical_system_parameters(new ICRA2009CanonicalSystemParameters());
  ICRA2009CSStatePtr icra2009_canonical_system_state(new ICRA2009CanonicalSystemState());
  ICRA2009CSPtr icra2009_canonical_system(new ICRA2009CanonicalSystem());
  if (!icra2009_canonical_system->initialize(icra2009_canonical_system_parameters, icra2009_canonical_system_state))
  {
    Logger::logPrintf("Could not initialize canonical system.", Logger::ERROR);
    return false;
  }

  DMPParamPtr dmp_parameters(new DynamicMovementPrimitiveParameters());
  if(!dmp_parameters->setCutoff(0.001))
  {
    Logger::logPrintf("Could not set cutoff.", Logger::ERROR);
    return false;
  }

  if (!DynamicMovementPrimitive::initialize(dmp_parameters, icra2009_transformation_systems, icra2009_canonical_system))
  {
    Logger::logPrintf("Could not initialize dmp.", Logger::ERROR);
    return false;
  }
  return (initialized_ = true);
}

}

