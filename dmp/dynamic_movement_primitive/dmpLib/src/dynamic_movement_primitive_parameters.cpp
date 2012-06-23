/*********************************************************************
  Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		parameter.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 4, 2010

 *********************************************************************/

// system include
#include <stdio.h>

// local include
#include <dmp_lib/dynamic_movement_primitive_parameters.h>
#include <dmp_lib/logger.h>

namespace dmp_lib
{

bool DynamicMovementPrimitiveParameters::initialize(const Time& initial_time,
                                                    const double teaching_duration,
                                                    const double execution_duration,
                                                    const double cutoff,
                                                    const int type,
                                                    const int id)
{
  // TODO: think whether to check if initial_time has a non-zero value
  initial_time_ = initial_time;

  if (teaching_duration < 0)
  {
    Logger::logPrintf("Teaching duration >%f< is invalid.", Logger::ERROR, teaching_duration);
    return false;
  }
  teaching_duration_ = teaching_duration;

  if (execution_duration < 0)
  {
    Logger::logPrintf("Execution duration >%f< is invalid.", Logger::ERROR, execution_duration);
    return false;
  }
  execution_duration_ = execution_duration;
  type_ = type;
  id_ = id;
  return setCutoff(cutoff);
}

bool DynamicMovementPrimitiveParameters::setCutoff(const double cutoff)
{
  if (cutoff < 1e-10)
  {
    Logger::logPrintf("Invalid cutoff specified >%f<.", Logger::ERROR, cutoff);
    return false;
  }
  cutoff_ = cutoff;
  return true;
}

bool DynamicMovementPrimitiveParameters::isCompatible(const DynamicMovementPrimitiveParameters& other_parameters) const
{
  if(initial_time_ != other_parameters.initial_time_)
  {
    Logger::logPrintf("Initial time is not compatible (dt=>%.4f<,tau=>%.4f<) vs (dt=>%.4f<,tau=>%.4f<)).",
                      Logger::ERROR, initial_time_.getDeltaT(), initial_time_.getTau(), other_parameters.initial_time_.getDeltaT(), other_parameters.initial_time_.getTau());
    return false;
  }
  if(cutoff_ != other_parameters.cutoff_)
  {
    Logger::logPrintf("Cutoffs >%f< and >%f< are not compatible.", Logger::ERROR, cutoff_, other_parameters.cutoff_);
    return false;
  }
  //  if(type_ != other_parameters.type_)
  //  {
  //    Logger::logPrintf("Types >%i< and >%i< are not compatible.", Logger::ERROR, type_, other_parameters.type_);
  //    return false;
  //  }
  return true;
}

bool DynamicMovementPrimitiveParameters::changeType(const DynamicMovementPrimitiveParameters& other_parameters)
{
  // TODO: change this !!!
  // only change type if they are different and not greater than 3
  if((type_ != other_parameters.type_) && (type_ < 3) && (other_parameters.type_ < 3))
  {
    type_ += other_parameters.type_;
  }
  return true;
}

bool DynamicMovementPrimitiveParameters::get(Time& initial_time,
                                             double& teaching_duration,
                                             double& execution_duration,
                                             double& cutoff,
                                             int& type,
                                             int &id) const
{
  initial_time = initial_time_;
  teaching_duration = teaching_duration_;
  execution_duration = execution_duration_;
  cutoff = cutoff_;
  type = type_;
  id = id_;
  return true;
}

}
