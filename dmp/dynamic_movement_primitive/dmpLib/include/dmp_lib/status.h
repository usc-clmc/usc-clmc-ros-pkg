/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		status.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 4, 2011

 *********************************************************************/

#ifndef DMP_STATUS_H_
#define DMP_STATUS_H_

// system includes

// local includes

namespace dmp_lib
{

class Status
{

public:

  /*! Constructor
   */
  Status() :
    initialized_(false) {};

  /*! Destructor
   */
  virtual ~Status() {};

  /*!
   * @return True if initialized, otherwise False
   */
  bool isInitialized() const;

protected:

  /*!
   */
  bool initialized_;

};

// Inline functions follow
inline bool Status::isInitialized() const
{
  return initialized_;
}

}

#endif /* DMP_STATUS_H_ */
