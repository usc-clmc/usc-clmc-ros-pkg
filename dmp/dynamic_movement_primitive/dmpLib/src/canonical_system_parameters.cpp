/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		canonical_system_parameters.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

// system includes
#include <math.h>
#include <stdio.h>

// local includes
#include <dmp_lib/canonical_system_parameters.h>
#include <dmp_lib/logger.h>

namespace dmp_lib
{

bool CanonicalSystemParameters::initialize(const double alpha_x)
{
  if (alpha_x > 1e-10)
  {
    alpha_x_ = alpha_x;
    return (initialized_ = true);
  }
  Logger::logPrintf("Canonical system parameter alpha_x >%f< must be possitive. Cannot initialize canonical system.", Logger::ERROR, alpha_x);
  return (initialized_ = false);
}

bool CanonicalSystemParameters::isCompatible(const CanonicalSystemParameters& other_parameters) const
{
  if(!initialized_)
  {
    Logger::logPrintf("Canonical system parameters not initialzed, therefore not compatible.", Logger::ERROR);
    return false;
  }
  if(!other_parameters.initialized_)
  {
    Logger::logPrintf("Other canonical system parameters not initialzed, therefore not compatible.", Logger::ERROR);
    return false;
  }
  if(alpha_x_ != other_parameters.alpha_x_)
  {
    Logger::logPrintf("Canonical system parameters not compatible.", Logger::ERROR);
    return false;
  }
  return true;
}

bool CanonicalSystemParameters::setCutoff(const double cutoff)
{
  if (cutoff > 1e-10)
  {
    alpha_x_ = -log(cutoff);
    return (initialized_ = true);
  }
  Logger::logPrintf("Invalid cutoff value provided >%f<.", Logger::ERROR, cutoff);
  return (initialized_ = false);
}

}
