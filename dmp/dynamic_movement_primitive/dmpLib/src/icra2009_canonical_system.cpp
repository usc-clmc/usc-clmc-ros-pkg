/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks    ...
 
  \file   icra2009_canonical_system.cpp

  \author Peter Pastor, Mrinal Kalakrishnan
  \date   Nov 6, 2010

 *********************************************************************/

// system includes
#include <math.h>
#include <stdio.h>
#include <cassert>

// local includes
#include <dmp_lib/icra2009_canonical_system.h>
#include <dmp_lib/logger.h>
#include <dmp_lib/utilities.h>

using namespace Eigen;

namespace dmp_lib
{

ICRA2009CanonicalSystem& ICRA2009CanonicalSystem::operator=(const ICRA2009CanonicalSystem& icra2009cs)
{
  Logger::logPrintf("ICRA2009CanonicalSystem assignment.", Logger::DEBUG);

  // first assign all memeber variables
  assert(Utilities<ICRA2009CanonicalSystemParameters>::assign(parameters_, icra2009cs.parameters_));
  assert(Utilities<ICRA2009CanonicalSystemState>::assign(state_, icra2009cs.state_));

  // then assign base class variables
  CanonicalSystem::parameters_ = parameters_;
  CanonicalSystem::state_ = state_;

  initialized_ = icra2009cs.initialized_;
  return *this;
}

bool ICRA2009CanonicalSystem::initialize(const ICRA2009CSParamPtr parameters,
                                         const ICRA2009CSStatePtr state)
{
  Logger::logPrintf("Initializing ICRA2009 canonical system.", Logger::DEBUG);

  // first set all memeber variables
  assert(Utilities<ICRA2009CanonicalSystemParameters>::assign(parameters_, parameters));
  assert(Utilities<ICRA2009CanonicalSystemState>::assign(state_, state));

  // then initialize base class variables
  return CanonicalSystem::initialize(parameters_, state_);
}

bool ICRA2009CanonicalSystem::get(ICRA2009CSParamConstPtr& parameters,
                                  ICRA2009CSStateConstPtr& state) const
{
  if(!initialized_)
  {
    Logger::logPrintf("Canonical system not initialized, cannot return parameters and state.", Logger::ERROR);
    return false;
  }
  parameters = parameters_;
  state = state_;
  return true;
}

void ICRA2009CanonicalSystem::reset()
{
  assert(initialized_);
  state_->setStateX(1.0);
  state_->setTime(0.0);
  state_->setProgressTime(0.0);
}

// REAL-TIME REQUIREMENTS
bool ICRA2009CanonicalSystem::integrate(const Time& dmp_time)
{
  // update state
  state_->setStateX(exp(-(parameters_->getAlphaX() / dmp_time.getTau()) * state_->time_));
  state_->addTime(dmp_time.getDeltaT());
  state_->setCanX(state_->getStateX());
  return true;
}

// REAL-TIME REQUIREMENTS
double ICRA2009CanonicalSystem::getTime() const
{
  return state_->time_;
}

bool ICRA2009CanonicalSystem::getRollout(const int num_time_steps, const double cutoff, VectorXd& rollout) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Canonical system not initialized, cannot set rollout.", Logger::ERROR);
    return false;
  }
  if(num_time_steps <= 0)
  {
    Logger::logPrintf("Number of time steps >%i< is invalid. Cannot return canonical system rollout.", Logger::ERROR, num_time_steps);
    return false;
  }
  if(cutoff < 1e-10)
  {
    Logger::logPrintf("Invalid cutoff value provided >%f<. Cannot return canonical system rollout.", Logger::ERROR, cutoff);
    return false;
  }

  if(rollout.size() != num_time_steps)
  {
    rollout.resize(num_time_steps);
  }
  rollout.setZero(num_time_steps);

  double dt = 1.0 / num_time_steps;
  double time = 0.0;
  double alpha_x = -log(cutoff);
  for (int i = 0; i < num_time_steps; ++i)
  {
    rollout(i) = exp(-(alpha_x) * time);
    time += dt;
  }
  return true;
}

}
