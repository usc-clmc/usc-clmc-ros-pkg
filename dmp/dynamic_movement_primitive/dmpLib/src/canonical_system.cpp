/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		canonical_system.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 4, 2010

 *********************************************************************/

// system include
#include <stdio.h>

// local include
#include <dmp_lib/canonical_system.h>
#include <dmp_lib/logger.h>
#include <dmp_lib/utilities.h>

namespace dmp_lib
{

bool CanonicalSystem::initialize(const CSParamPtr parameters,
                                 const CSStatePtr state)
{
  Logger::logPrintf("Initializing canonical system.", Logger::DEBUG);
  if (!parameters.get())
  {
    Logger::logPrintf("Cannot initialize canonical system from empty parameters.", Logger::ERROR);
    return false;
  }
  if (!state.get())
  {
    Logger::logPrintf("Cannot initialize canonical system from empty state.", Logger::ERROR);
    return false;
  }
  Logger::logPrintf(initialized_, "Canonical system already initialized. Re-initializing...", Logger::WARN);
  parameters_ = parameters;
  state_ = state;
  return (initialized_ = true);
}

bool CanonicalSystem::isCompatible(const CanonicalSystem& other_cs) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Canonical system not initialized, not compatible.", Logger::ERROR);
    return false;
  }
  if (!other_cs.initialized_)
  {
    Logger::logPrintf("Other canonical system not initialized, not compatible.", Logger::ERROR);
    return false;
  }
  return (parameters_->isCompatible(*other_cs.parameters_) && state_->isCompatible(*other_cs.state_));
}

void CanonicalSystem::reset()
{
  assert(initialized_);
  // start from the beginning
  state_->setStateX(1.0);
  state_->setTime(0.0);
  state_->setProgressTime(0.0);
}

void CanonicalSystem::reset(const double cutoff,
                            const double movement_duration,
                            const double progress_start)
{
  assert(initialized_);
  assert(!(progress_start < 0.0));
  assert(!(progress_start > 1.0));
  double x = 1.0 - progress_start;
  if (x < cutoff)
    x = cutoff;
  state_->setStateX(x);
  state_->setTime(progress_start * movement_duration);
  state_->setProgressTime(progress_start * movement_duration);
}

// REAL-TIME REQUIREMENTS
double CanonicalSystem::getTime() const
{
  return state_->time_;
}

bool CanonicalSystem::get(CSParamConstPtr& parameters,
                          CSStateConstPtr& state) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Canonical system is not initialized, cannot return parameters.", Logger::ERROR);
    return false;
  }
  parameters = parameters_;
  state = state_;
  return true;
}

}
