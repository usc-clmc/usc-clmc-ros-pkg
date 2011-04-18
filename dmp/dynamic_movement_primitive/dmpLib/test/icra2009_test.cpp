/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_test.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Feb 20, 2011

 *********************************************************************/

// system includes
#include <string>
#include <vector>
#include <stdio.h>

#include <dmp_lib/icra2009_dynamic_movement_primitive.h>
#include <dmp_lib/logger.h>

// local includes
#include "icra2009_test.h"

using namespace dmp_lib;
using namespace std;

// import most common Eigen types
using namespace Eigen;

namespace test_dmp
{

bool ICRA2009Test::initialize(ICRA2009DMP& dmp, const TestData& testdata)
{

  lwr_lib::LWRParamPtr lwr_parameters(new lwr_lib::LWRParameters());
  if (!lwr_parameters->initialize(testdata.getNumRFS(), testdata.getActivation()))
  {
    dmp_lib::Logger::logPrintf("Could not initialize LWR parameters.", dmp_lib::Logger::ERROR);
    return false;
  }

  lwr_lib::LWRPtr lwr_model(new lwr_lib::LWR());
  if (!lwr_model->initialize(lwr_parameters))
  {
    Logger::logPrintf("Could not initialize LWR model with provide parameters.", Logger::ERROR);
    return false;
  }

  vector<ICRA2009TSPtr> icra2009_transformation_systems;

  vector<string> variable_names = testdata.getTestVariableNames();
  for (int i = 0; i < (int)variable_names.size(); ++i)
  {
    vector<ICRA2009TSStatePtr> icra2009_transformation_system_state_vector;
    vector<ICRA2009TSParamPtr> icra2009_transformation_system_parameters_vector;

    ICRA2009TSParamPtr icra2009_transformation_system_parameters;
    ICRA2009TSStatePtr icra2009_transformation_system_state;

    if(testdata.getTestCase() == TestData::QUAT_TEST && testdata.getQuatTSIndex() == i)
    {
      if(i+4 > (int)variable_names.size())
      {
        Logger::logPrintf("Number of variable names is invalid.", Logger::ERROR);
        return false;
      }
      for (int j = 0; j < 4; ++j)
      {
        icra2009_transformation_system_parameters.reset(new ICRA2009TransformationSystemParameters());
        if (!icra2009_transformation_system_parameters->initialize(lwr_model, variable_names[i+j], testdata.getKGain(), testdata.getDGain()))
        {
          Logger::logPrintf("Could not initialize transformation system parameters.", Logger::ERROR);
          return false;
        }
        icra2009_transformation_system_parameters_vector.push_back(icra2009_transformation_system_parameters);

        icra2009_transformation_system_state.reset(new ICRA2009TransformationSystemState());
        icra2009_transformation_system_state_vector.push_back(icra2009_transformation_system_state);
      }
      i+=4;
    }
    else
    {
      icra2009_transformation_system_parameters.reset(new ICRA2009TransformationSystemParameters());
      if (!icra2009_transformation_system_parameters->initialize(lwr_model, variable_names[i], testdata.getKGain(), testdata.getDGain()))
      {
        Logger::logPrintf("Could not initialize transformation system parameters.", Logger::ERROR);
        return false;
      }
      icra2009_transformation_system_parameters_vector.push_back(icra2009_transformation_system_parameters);

      icra2009_transformation_system_state.reset(new ICRA2009TransformationSystemState());
      icra2009_transformation_system_state_vector.push_back(icra2009_transformation_system_state);
    }

    // TODO: fix the integration method
    ICRA2009TSPtr icra2009_transformation_system(new ICRA2009TransformationSystem());
    if (!icra2009_transformation_system->initialize(icra2009_transformation_system_parameters_vector,
                                                    icra2009_transformation_system_state_vector,
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

  ICRA2009DMPParamPtr icra2009_dmp_parameters(new ICRA2009DynamicMovementPrimitiveParameters());
  if(!icra2009_dmp_parameters->setCutoff(testdata.getCutoff()))
  {
    Logger::logPrintf("Could not set cutoff.", Logger::ERROR);
    return false;
  }
  ICRA2009DMPStatePtr icra2009_dmp_state(new ICRA2009DynamicMovementPrimitiveState());

  if(!dmp.initialize(icra2009_dmp_parameters, icra2009_dmp_state, icra2009_transformation_systems, icra2009_canonical_system))
  {
    Logger::logPrintf("Could not initialize icra2009 dmp.", Logger::ERROR);
    return false;
  }

  if(testdata.getTestCase() == TestData::QUAT_TEST)
  {
    dmp.getTransformationSystem(testdata.getQuatTSIndex())->setIntegrationMethod(TransformationSystem::QUATERNION);
  }

  return true;
}

}
