/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		status.h

  \author	Peter Pastor
  \date		Jan 4, 2011

 *********************************************************************/

#ifndef LWR_STATUS_H_
#define LWR_STATUS_H_

// system includes

// local includes

namespace lwr_lib
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

#endif /* LWR_STATUS_H_ */
