/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		test_trajectory.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Nov 23, 2010

 *********************************************************************/

#ifndef TEST_TRAJECTORY_H_
#define TEST_TRAJECTORY_H_

// system includes

// local includes
#include <dmp_lib/trajectory.h>
#include "test_data.h"

namespace test_dmp
{

/*!
 */
class TestTrajectory
{

public:

    /*!
     */
    static bool test(const std::string& filename, const TestData& testdata, const std::string = "");

private:

    /*!
     */
    TestTrajectory() {};
    /*!
     */
    virtual ~TestTrajectory() {};

};

}


#endif /* TEST_TRAJECTORY_H_ */
