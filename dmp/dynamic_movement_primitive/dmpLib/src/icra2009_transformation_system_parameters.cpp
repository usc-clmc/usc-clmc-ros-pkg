/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_transformation_system_parameters.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 6, 2010

 *********************************************************************/

// system includes
#include <stdio.h>

// local includes
#include <dmp_lib/icra2009_transformation_system_parameters.h>
#include <dmp_lib/logger.h>

namespace dmp_lib
{

bool ICRA2009TransformationSystemParameters::initialize(const double k_gain, const double d_gain)
{
  if (k_gain < 0)
  {
    Logger::logPrintf("Invalid K gain >%f<. Cannot initialize ICRA2009 transformation system.", Logger::ERROR, k_gain);
    return false;
  }
  k_gain_ = k_gain;
  if (d_gain < 0)
  {
    Logger::logPrintf("Invalid D gain >%f<. Cannot initialize ICRA2009 transformation system.", Logger::ERROR, d_gain);
    return false;
  }
  d_gain_ = d_gain;
  return true;
}

bool ICRA2009TransformationSystemParameters::get(double& k_gain,
                                                 double& d_gain) const
{
  // TODO: think about error/init checking...
  k_gain = k_gain_;
  d_gain = d_gain_;
  return true;
}

bool ICRA2009TransformationSystemParameters::initialize(const lwr_lib::LWRPtr lwr_model,
                                                        const std::string& name,
                                                        const double k_gain, const double d_gain,
                                                        const double initial_start, const double initial_goal)
{
  if (!TransformationSystemParameters::initialize(lwr_model, name, initial_start, initial_goal))
  {
    Logger::logPrintf("Could not initialize ICRA2009 transformation system.", Logger::ERROR);
    return false;
  }
  if (!initialize(k_gain, d_gain))
  {
    return false;
  }
  return true;
}

}
