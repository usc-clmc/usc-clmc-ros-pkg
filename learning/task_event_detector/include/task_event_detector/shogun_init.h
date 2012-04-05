/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		shogun_init.h

  \author	Peter Pastor
  \date		Jul 8, 2011

 *********************************************************************/

#ifndef SHOGUN_INIT_H_
#define SHOGUN_INIT_H_

// system includes
#include <shogun/base/init.h>

// local includes

namespace task_event_detector
{

void printMessage(FILE* target,
                  const char* str)
{
  // ROS_INFO("%s", str);
  printf("%s", str);
  // fprintf(target, "%s", str);
}

void printWarning(FILE* target,
                  const char* str)
{
  // ROS_WARN("%s", str);
  printf("%s", str);
  // fprintf(target, "%s", str);
}

void printError(FILE* target,
                const char* str)
{
  // ROS_ERROR("%s", str);
  printf("%s", str);
  // fprintf(target, "%s", str);
}

void cancelComputations(bool &delayed,
                        bool &immediately)
{
  ROS_INFO_COND(delayed, "Cancel computation called with delayed = true.");
  // ROS_INFO_COND(!delayed, "Cancel computation called with delayed = false.");
  ROS_INFO_COND(immediately, "Cancel computation called with immediately = true.");
  // ROS_INFO_COND(!immediately, "Cancel computation called with immediately = false.");
}

void init()
{
  shogun::init_shogun(&printMessage, &printWarning, &printError, &cancelComputations);
}

void exit()
{
  shogun::exit_shogun();
}

}

#endif /* SHOGUN_INIT_H_ */
