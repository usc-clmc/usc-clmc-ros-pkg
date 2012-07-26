/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		state.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Dec 7, 2010

 *********************************************************************/

#ifndef STATE_BASE_H_
#define STATE_BASE_H_

// system includes
#include <math.h>

// local includes

namespace dmp_lib
{

/*!
 */
class State
{

public:

    /*! Constructor
     */
    State() :
      x_(0), xd_(0), xdd_(0) {};
    State(const double x, const double xd, const double xdd) :
      x_(x), xd_(xd), xdd_(xdd) {};

    /*! Destructor
     */
    virtual ~State(){};

    /*!
      * @param state
      * @return True if equal, otherwise False
      */
    bool operator==(const State &state) const
    {
      return ( (fabs(x_ - state.x_) < EQUALITY_PRECISSION)
          && (fabs(xd_ - state.xd_) < EQUALITY_PRECISSION)
          && (fabs(xdd_ - state.xdd_) < EQUALITY_PRECISSION) );
    }
    bool operator!=(const State &state) const
    {
      return !(*this == state);
    }

    /*!
     */
    void reset();

    /*!
     * @param x
     * @param xd
     * @param xdd
     */
    void set(const double x, const double xd, const double xdd);

    /*!
     * @param x
     * @param xd
     * @param xdd
     */
    void get(double& x, double& xd, double& xdd);

    /*!
     * @return
     */
    double getX() const;

    /*!
     * @param x
     */
    void setX(const double x);

    /*!
     * @param x
     */
    void addX(const double x);

    /*!
     * @return
     */
    double getXd() const;

    /*!
     * @param xd
     */
    void setXd(const double xd);

    /*!
     * @param xd
     */
    void addXd(const double xd);

    /*!
     * @return
     */
    double getXdd() const;

    /*!
     * @param xdd
     */
    void setXdd(const double xdd);

protected:

    static const double EQUALITY_PRECISSION = 1e-6;
    double x_, xd_, xdd_;

};

// inline functions
inline void State::set(const double x, const double xd, const double xdd)
{
    x_ = x;
    xd_ = xd;
    xdd_ = xdd;
}
inline void State::get(double& x, double& xd, double& xdd)
{
    x = x_;
    xd = xd_;
    xdd = xdd_;
}

inline double State::getX() const
{
    return x_;
}
inline void State::setX(const double x)
{
    x_ = x;
}
inline void State::addX(const double x)
{
    x_ += x;
}
inline double State::getXd() const
{
    return xd_;
}
inline void State::setXd(const double xd)
{
    xd_ = xd;
}
inline void State::addXd(const double xd)
{
    xd_ += xd;
}
inline double State::getXdd() const
{
    return xdd_;
}
inline void State::setXdd(const double xdd)
{
    xdd_ = xdd;
}
inline void State::reset()
{
    x_ = 0;
    xd_ = 0;
    xdd_ = 0;
}


}

#endif /* STATE_BASE_H_ */
