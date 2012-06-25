/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		test.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Feb 20, 2011

 *********************************************************************/

#ifndef TEST_H_
#define TEST_H_

// system includes
#include <string>
#include <vector>
#include <stdio.h>

#include <dmp_lib/logger.h>
#include <dmp_lib/icra2009_dynamic_movement_primitive.h>
#include <dmp_lib/nc2010_dynamic_movement_primitive.h>

// local includes
#include "test_dynamic_movement_primitive.h"
#include "test_trajectory.h"
#include "test_data.h"
#include "icra2009_test.h"

#include <unistd.h>

namespace test_dmp
{

class Test
{

public:

  static bool test(const std::string base_directory = "", const bool regenerate_result_data = false);

private:

  Test(){};
  virtual ~Test(){};

};

inline bool Test::test(const std::string base_directory, const bool regenerate_result_data)
{
  TestData testdata;
  if (!testdata.initialize(TestData::QUAT_TEST))
  {
    dmp_lib::Logger::logPrintf("Could not initialize test data.");
    return false;
  }

  for (int i = 0; i < testdata.getNumTests(); ++i)
  {
    dmp_lib::ICRA2009DMP dmp;
    if (!test_dmp::ICRA2009Test::initialize(dmp, testdata))
    {
      dmp_lib::Logger::logPrintf("Could not initialize ICRA2009 DMP. Test failed.", dmp_lib::Logger::ERROR);
      return false;
    }

    std::vector<double> goaloffsets;
    if (!testdata.getGoal(i, goaloffsets))
    {
      dmp_lib::Logger::logPrintf("Test data invalid.", dmp_lib::Logger::ERROR);
      return false;
    }
    double duration_scale;
    if (!testdata.getDurationScale(i, duration_scale))
    {
      dmp_lib::Logger::logPrintf("Test data invalid.", dmp_lib::Logger::ERROR);
      return false;
    }
    double error_threshold;
    if (!testdata.getErrorThreshold(i, error_threshold))
    {
      dmp_lib::Logger::logPrintf("Test data invalid.", dmp_lib::Logger::ERROR);
      return false;
    }

    if (!test_dmp::TestDMP<dmp_lib::ICRA2009DMP>::test(dmp, std::string(testdata.getTestTrajectoryFileName()), std::string("_icra2009"), testdata.getNumDataPoints(),
                                              goaloffsets, duration_scale, error_threshold, i, base_directory, regenerate_result_data))
    {
      dmp_lib::Logger::logPrintf("ICRA2009 dmp test failed.", dmp_lib::Logger::ERROR);
      return false;
    }
  }

  if (!testdata.initialize(TestData::SIMPLE_TEST))
  {
    dmp_lib::Logger::logPrintf("Could not initialize test data.");
    return false;
  }

  if (!test_dmp::TestTrajectory::test("test_trajectory_class", testdata, base_directory))
  {
    dmp_lib::Logger::logPrintf("Trajectory test failed.", dmp_lib::Logger::ERROR);
    return false;
  }

  dmp_lib::Logger::logPrintf("Test finished successful.", dmp_lib::Logger::INFO);
  return true;
}

}

#endif /* TEST_H_ */
