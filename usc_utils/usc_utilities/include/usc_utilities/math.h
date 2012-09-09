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
double sigmoid(double x, double center, double width, double& gradient);

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

inline double sigmoid(double x, double center, double width, double& gradient)
{
  // width of a "standard" sigmoid is 4
  double slope = 4.0/width; // slope of 4 makes the width 1
  double temp = exp(-slope *(center-x));
  double y = temp / (1.0 + temp)
  gradient = (1.0 - y)*y;
}

#endif /* USC_UTILITIES_MATH_H_ */
