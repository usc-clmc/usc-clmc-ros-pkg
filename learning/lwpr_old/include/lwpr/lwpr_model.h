/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    lwpr_model.h

 \author  Peter Pastor
 \date    June 4, 2011

 *********************************************************************/

#ifndef LWPR_MODEL_H_
#define LWPR_MODEL_H_

// system includes
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <lwpr_lib/lwpr.hh>

// local includes

namespace lwpr
{

class LWPRModel
{

public:

  /*! Constructor
   */
  LWPRModel();
  /*! Destructor
   */
  virtual ~LWPRModel() {};

  /*!
   * @return True on success, False otherwise
   */
  bool initialize(ros::NodeHandle node_handle, const int index = 1);
  bool isInitialized() const;

  /*!
   * @param file_name
   * @return True on success, False otherwise
   */
  bool writeToDisc(const std::string& file_name);

  /*! \brief Updates an LWPR model with a given input/output pair (x,y).
   * @param x input
   * @param y output
   * @param prediction Current prediction of y given x, useful for tracking the training error.
   * @return True on success, False otherwise
   */
  bool update(const double& x,
              const double& y,
              double& prediction);

  /*! \brief Computes the prediction of an LWPR model given an input vector x.
   *
   * @param x input
   * @param y output
   * @return True on success, False otherwise
   */
  bool predict(const double& x,
               double& y);

  /*! \brief Computes the prediction of an LWPR model given an input vector x.
   *
   * @param x input
   * @param y output
   * @return True on success, False otherwise
   */
  bool predict(const double& x,
               double& y,
               double& conf);

private:

  bool initialized_;
  ros::NodeHandle node_handle_;

  int index_;
  boost::shared_ptr<lwpr_lib::LWPR_Object> lwpr_object_;

  struct LWPRParameters
  {
    double cutoff_;
    lwpr_lib::doubleVec input_;
    lwpr_lib::doubleVec output_;
    lwpr_lib::doubleVec prediction_;
    lwpr_lib::doubleVec confidence_;
  };
  LWPRParameters parameters_;

};

// inline functions follow

}

#endif /* LWPR_MODEL_H_ */
