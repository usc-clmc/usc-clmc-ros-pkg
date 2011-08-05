/*
 * timer.h
 *
 *  Created on: Nov 19, 2010
 *      Author: kalakris
 */

#ifndef USC_UTILITIES_TIMER_H_
#define USC_UTILITIES_TIMER_H_

#ifdef __XENO__
#include <native/timer.h>
#else
#include <sys/time.h>
#endif

namespace usc_utilities
{

/**
 * Provides a timer class to measure time in real-time code or outside.
 */
class Timer
{
public:

  void startTimer();
  double getElapsedTimeMicroSeconds() const;
  double getElapsedTimeMilliSeconds() const;
  double getElapsedTimeSeconds() const;

private:
#ifdef __XENO__
  RTIME start_time_;
#else
  struct timeval start_time_;

  /**
   * Performs diff = x - y.
   * @param diff
   * @return true if difference is positive, false otherwise
   */
  static bool timevalSubtract(const struct timeval& x, const struct timeval& y, struct timeval& diff);

#endif

};

/////////////////////// inline functions follow ///////////////////////

inline void Timer::startTimer()
{
#ifdef __XENO__
  start_time_ = rt_timer_read();
#else
  gettimeofday(&start_time_, NULL);
#endif
}

inline double Timer::getElapsedTimeMicroSeconds() const
{
#ifdef __XENO__
  RTIME end_time = rt_timer_read();
  return (end_time - start_time_)/1000.0;
#else
  struct timeval end_time;
  gettimeofday(&end_time, NULL);
  struct timeval interval;
  timevalSubtract(end_time, start_time_, interval);
  double time = interval.tv_sec * 1000000.0 + interval.tv_usec;
  return time;
#endif
}

inline double Timer::getElapsedTimeMilliSeconds() const
{
  return getElapsedTimeMicroSeconds()/1000.0;
}

inline double Timer::getElapsedTimeSeconds() const
{
  return getElapsedTimeMicroSeconds()/1000000.0;
}

#ifndef __XENO__
inline bool Timer::timevalSubtract(const struct timeval& x, const struct timeval& y_orig, struct timeval& diff)
{
  struct timeval y = y_orig;

  // Perform the carry for the later subtraction by updating y.
  if (x.tv_usec < y.tv_usec) {
    int nsec = (y.tv_usec - x.tv_usec) / 1000000 + 1;
    y.tv_usec -= 1000000 * nsec;
    y.tv_sec += nsec;
  }
  if (x.tv_usec - y.tv_usec > 1000000) {
    int nsec = (x.tv_usec - y.tv_usec) / 1000000;
    y.tv_usec += 1000000 * nsec;
    y.tv_sec -= nsec;
  }

  // Compute the time remaining to wait. tv_usec is certainly positive.
  diff.tv_sec = x.tv_sec - y.tv_sec;
  diff.tv_usec = x.tv_usec - y.tv_usec;

  // Return false if result is negative.
  return x.tv_sec > y.tv_sec;
}
#endif

}
#endif /* USC_UTILITIES_TIMER_H_ */
