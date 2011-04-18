/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    logger.h

 \author  Mrinal Kalakrishnan, Jonas Buchli, Peter Pastor
 \date    Jan 10, 2011

 *********************************************************************/
#ifndef LOGGER_H_
#define LOGGER_H_

// system includes
#include <string>
#include <stddef.h>
#include <stdio.h>

namespace lwr_lib
{

class Logger
{

public:

  enum Level
  {
    DEBUG = 1, INFO, WARN, ERROR, FATAL
  };

  /*!
   * @param msg
   * @param level
   * @return
   */
  static void logPrintf(const char *msg,
                        Logger::Level level = Logger::INFO,
                        ...);

  /*!
   * @param condition
   * @param msg
   * @param level
   * @return
   */
  static void logPrintf(bool condition,
                        const char *msg,
                        Logger::Level level = Logger::INFO,
                        ...);

  /*!
   * @param level
   */
  static void setLogLevel(Logger::Level level)
  {
    level_ = level;
  }

  /*!
   * @param level
   * @return
   */
  static std::string getLogLevel(Logger::Level level);

private:

  /*! Constructor
   */
  Logger() {};

  /*! Destructor
   */
  virtual ~Logger() {};

  /*!
   * @param level
   * @return
   */
  static bool toBeLogged(Level level)
  {
    return (level >= level_);
  }

  static Level level_;

};

// Inline functions follow
inline std::string Logger::getLogLevel(Logger::Level level)
{
  std::string level_string;
  switch (level)
  {
    case DEBUG:
      level_string.assign("DEBUG");
      break;
    case INFO:
      level_string.assign("INFO");
      break;
    case WARN:
      level_string.assign("WARN");
      break;
    case ERROR:
      level_string.assign("ERROR");
      break;
    case FATAL:
      level_string.assign("FATAL");
      break;
  }
  return level_string;
}

}

#endif // LOGGER_H_
