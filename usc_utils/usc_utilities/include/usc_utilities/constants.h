/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		constants.h

 \author	Peter Pastor
 \date		Jan 14, 2011

 *********************************************************************/

#ifndef UTILITIES_CONSTANTS_H_
#define UTILITIES_CONSTANTS_H_

// system includes

// local includes

namespace usc_utilities
{

class Constants
{

public:

  enum Position
  {
    X = 0, Y, Z, N_CART
  };

  enum Orientation
  {
    QW = 0, QX, QY, QZ, N_QUAT
  };

  enum PosVelAcc
  {
    POS = 0, VEL, ACC, POS_VEL_ACC
  };

private:

  /*! Constructor
   */
  Constants() {};

  /*! Destructor
   */
  virtual ~Constants() {};

};

}

#endif /* UTILITIES_CONSTANTS_H_ */
