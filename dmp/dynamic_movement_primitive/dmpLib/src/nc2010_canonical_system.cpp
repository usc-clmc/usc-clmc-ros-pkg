/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		nc2010_canonical_system.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 6, 2010

 *********************************************************************/

// system includes
#include <math.h>
#include <stdio.h>
#include <cassert>

// local includes
#include <dmp_lib/nc2010_canonical_system.h>
#include <dmp_lib/logger.h>
#include <dmp_lib/utilities.h>

using namespace Eigen;

namespace dmp_lib
{

NC2010CanonicalSystem& NC2010CanonicalSystem::operator=(const NC2010CanonicalSystem& nc2010cs)
{
  Logger::logPrintf("NC2010CanonicalSystem assignment.", Logger::DEBUG);

  // first assign all memeber variables
  assert(Utilities<NC2010CanonicalSystemParameters>::assign(parameters_, nc2010cs.parameters_));
  assert(Utilities<NC2010CanonicalSystemState>::assign(state_, nc2010cs.state_));

  // then assign base class variables
  CanonicalSystem::parameters_ = parameters_;
  CanonicalSystem::state_ = state_;

  initialized_ = nc2010cs.initialized_;
  return *this;
}

bool NC2010CanonicalSystem::initialize(const NC2010CSParamPtr parameters,
                                         const NC2010CSStatePtr state)
{
  Logger::logPrintf("Initializing NC2010 canonical system.", Logger::DEBUG);

  // first set all memeber variables
  assert(Utilities<NC2010CanonicalSystemParameters>::assign(parameters_, parameters));
  assert(Utilities<NC2010CanonicalSystemState>::assign(state_, state));

  // then initialize base class variables
  return CanonicalSystem::initialize(parameters_, state_);
}

bool NC2010CanonicalSystem::get(NC2010CSParamConstPtr& parameters,
                                  NC2010CSStateConstPtr& state) const
{
  parameters = parameters_;
  state = state_;
  return true;
}

void NC2010CanonicalSystem::reset()
{
  state_->setStateX(1.0);
  state_->setTime(0.0);
}

bool NC2010CanonicalSystem::integrate(const Time& dmp_time)
{
  // update state
  state_->setStateX(exp(-(parameters_->getAlphaX() / dmp_time.getTau()) * state_->time_));
  state_->addTime(dmp_time.getDeltaT());
  return true;
}

double NC2010CanonicalSystem::getTime() const
{
  return state_->time_;
}


bool NC2010CanonicalSystem::getRollout(const int num_time_steps, const double cutoff, VectorXd& rollout) const
{

  return true;
}
}
