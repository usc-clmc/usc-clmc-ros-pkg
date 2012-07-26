/*
 * spline.h
 *
 *  Created on: Nov 11, 2010
 *      Author: kalakris
 */

#ifndef SPLINES_SPLINES_H_
#define SPLINES_SPLINES_H_

#include <vector>

namespace splines
{

class Spline
{
public:
  explicit Spline(int degree);
  virtual ~Spline();

  /**
   * Samples the spline at time=t
   *
   * @param t (in) time
   * @param x (out) position
   * @param xd (out) velocity
   * @param xdd (out) acceleration
   */
  virtual void sample(double t, double& x, double& xd, double& xdd) const=0;

  std::vector<double>& getCoefficients();

protected:
  std::vector<double> coefficients_;

  /**
   * Generates powers of x, from x^0 to x^n
   * @param n maximum power to generate
   * @param x
   * @param powers double array, must be of size >= n+1
   */
  static void generatePowers(int n, double x, double* powers);
};

class QuinticSpline: public Spline
{
public:
  QuinticSpline();
  virtual ~QuinticSpline();

  void setCoefficients(double x0, double x0d, double x0dd, double x1, double x1d, double x1dd, double time);
  virtual void sample(double t, double& x, double& xd, double& xdd) const;

};

// inline functions follow:

inline Spline::Spline(int degree):
    coefficients_(degree)
{

}

inline Spline::~Spline()
{

}

inline std::vector<double>& Spline::getCoefficients()
{
  return coefficients_;
}

inline void Spline::generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; ++i)
  {
    powers[i] = powers[i-1]*x;
  }
}

inline QuinticSpline::QuinticSpline():
    Spline(6)
{
}

inline QuinticSpline::~QuinticSpline()
{

}

inline void QuinticSpline::sample(double time, double& x, double& xd, double& xdd) const
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  x = t[0]*coefficients_[0] +
      t[1]*coefficients_[1] +
      t[2]*coefficients_[2] +
      t[3]*coefficients_[3] +
      t[4]*coefficients_[4] +
      t[5]*coefficients_[5];

  xd = t[0]*coefficients_[1] +
      2.0*t[1]*coefficients_[2] +
      3.0*t[2]*coefficients_[3] +
      4.0*t[3]*coefficients_[4] +
      5.0*t[4]*coefficients_[5];

  xdd = 2.0*t[0]*coefficients_[2] +
      6.0*t[1]*coefficients_[3] +
      12.0*t[2]*coefficients_[4] +
      20.0*t[3]*coefficients_[5];

}

inline void QuinticSpline::setCoefficients(double x0, double x0d, double x0dd, double x1, double x1d, double x1dd, double time)
{
  double T[6];
  generatePowers(5, time, T);

  coefficients_[0] = x0;
  coefficients_[1] = x0d;
  coefficients_[2] = 0.5*x0dd;
  coefficients_[3] = (-20.0*x0 + 20.0*x1 - 3.0*x0dd*T[2] + x1dd*T[2] -
      12.0*x0d*T[1] - 8.0*x1d*T[1]) / (2.0*T[3]);
  coefficients_[4] = (30.0*x0 - 30.0*x1 + 3.0*x0dd*T[2] - 2.0*x1dd*T[2] +
      16.0*x0d*T[1] + 14.0*x1d*T[1]) / (2.0*T[4]);
  coefficients_[5] = (-12.0*x0 + 12.0*x1 - x0dd*T[2] + x1dd*T[2] -
      6.0*x0d*T[1] - 6.0*x1d*T[1]) / (2.0*T[5]);
}

}

#endif /* SPLINE_H_ */
