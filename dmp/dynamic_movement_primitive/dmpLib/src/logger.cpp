/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    logger.cpp

 \author  Mrinal Kalakrishnan, Jonas Buchli, Peter Pastor, Mrinal Kalakrishnan
 \date    Jan 10, 2011

 *********************************************************************/

// system includes
#include "stdarg.h"

// local includes
#include <dmp_lib/logger.h>

namespace dmp_lib
{

Logger::Level Logger::level_ = Logger::WARN;

void Logger::logPrintf(const char *msg,
                       Logger::Level level,
                       ...)
{
  if (Logger::toBeLogged(level))
  {
    va_list va;
    va_start(va,level);
    printf("%s: ", Logger::getLogLevel(level).c_str());
    vprintf(msg, va);
    va_end(va);
    printf("\n");
  }
}

void Logger::logPrintf(bool condition,
                       const char *msg,
                       Logger::Level level,
                       ...)
{
  if (condition)
  {
    if (Logger::toBeLogged(level))
    {
      va_list va;
      va_start(va,level);
      printf("%s: ", Logger::getLogLevel(level).c_str());
      vprintf(msg, va);
      va_end(va);
      printf("\n");
    }
  }
}

}
