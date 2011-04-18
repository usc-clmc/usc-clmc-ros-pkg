/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		test_dynamic_movement_primitive.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 5, 2010

 *********************************************************************/

// local includes
#include "test.h"

int main(int argc, char** argv)
{
  std::string base_directory = "";
  bool regenerate_result_data = false;

  for(int i = 0; i < argc; i++)
  {
    std::string generate_test_data = "-g";
    if(generate_test_data.compare(argv[i]) == 0)
    {
      dmp_lib::Logger::logPrintf("Regenerating test data.", dmp_lib::Logger::INFO);
      regenerate_result_data = true;
    }
  }

  if(test_dmp::Test::test(base_directory, regenerate_result_data))
  {
    return -1;
  }
  return 0;
}
