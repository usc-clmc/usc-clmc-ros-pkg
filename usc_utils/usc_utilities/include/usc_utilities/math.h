/*
 * math.h
 *
 *  Created on: Jul 28, 2011
 *      Author: kalakris
 */

#ifndef USC_UTILITIES_MATH_H_
#define USC_UTILITIES_MATH_H_

#include <cmath>

namespace usc_utilities
{

double clipAbsoluteValue(double x, double max_absolute_value);

inline double clipAbsoluteValue(double x, double max_absolute_value)
{
  if (x>max_absolute_value)
  {
    return max_absolute_value;
  }
  else if (x<-max_absolute_value)
  {
    return -max_absolute_value;
  }
  else return x;
}

}

#endif /* USC_UTILITIES_MATH_H_ */
