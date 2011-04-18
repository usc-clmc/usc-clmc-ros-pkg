/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		icra2009_test.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Feb 20, 2011

 *********************************************************************/

#ifndef ICRA2009_TEST_H_
#define ICRA2009_TEST_H_

// system includes
#include <dmp_lib/icra2009_dynamic_movement_primitive.h>

// local includes
#include "test_data.h"

namespace test_dmp
{

class ICRA2009Test
{

public:

  static bool initialize(dmp_lib::ICRA2009DMP& dmp, const TestData& testdata);

private:

  ICRA2009Test() {};
  virtual ~ICRA2009Test() {};

};

}

#endif /* ICRA2009_TEST_H_ */
