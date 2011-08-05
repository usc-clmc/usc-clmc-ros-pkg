/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...

 \file		dmp_controller_implementation.h

 \author	Alexander Herzor, Peter Pastor
 \date		Jan 12, 2011

 *********************************************************************/

#ifndef DMP_DUAL_ARM_CONTROLLER_IMPLEMENTATION_H_
#define DMP_DUAL_ARM_CONTROLLER_IMPLEMENTATION_H_

// system includes
#include <string>
#include <vector>

// ros includes
#include <ros/ros.h>
#include <rosrt/rosrt.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/constants.h>
#include <usc_utilities/param_server.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>

// local includes
#include <pr2_dynamic_movement_primitive_controller/dmp_dual_arm_controller.h>

namespace pr2_dynamic_movement_primitive_controller
{

template<class DMPType>
  class DMPDualArmControllerImplementation : public DMPDualArmController
  {

  public:

    /*! Constructor
     */
    DMPDualArmControllerImplementation() {};

    /*! Destructor
     */
    virtual ~DMPDualArmControllerImplementation() {};

    /*!
     * @param controller_name
     * @return True on success, otherwise False
     */
    bool initialize(const std::string& controller_name);

    /*!
     * @return True on success, otherwise False
     * REAL-TIME REQUIREMENTS
     */
    bool newDMPReady();

  private:

    /*! Callback function to filter incoming messages
     * @param msg
     * @param dmp
     * @return
     */
    bool filter(const typename DMPType::DMPMsgConstPtr& msg,
                const typename DMPType::DMPPtr dmp);

    /*!
     * @param status
     * @param movement_finished
     * @param end_time
     * REAL-TIME REQUIREMENTS
     */
    void publishStatus(const int status, const bool movement_finished, const ros::Time& end_time);

    /*!
     */
    rosrt::FilteredSubscriber<typename DMPType::DMPMsg, typename DMPType::DMP> dmp_filtered_subscriber_;

    /*!
     */
    typename DMPType::DMPPtr dmp_;

  };

template<class DMPType>
  bool DMPDualArmControllerImplementation<DMPType>::filter(const typename DMPType::DMPMsgConstPtr& msg,
                                                    const typename DMPType::DMPPtr dmp)
  {
    return DMPType::initFromMessage(dmp, *msg);
  }

template<class DMPType>
  bool DMPDualArmControllerImplementation<DMPType>::initialize(const std::string& controller_name)
  {

    ros::NodeHandle node_handle;
    ROS_VERIFY(dmp_filtered_subscriber_.initialize(1000, node_handle, controller_name + "/command", boost::bind(&DMPDualArmControllerImplementation<DMPType>::filter, this, _1, _2)));

    ros::Publisher publisher = node_handle.advertise<dynamic_movement_primitive::ControllerStatusMsg>(controller_name + "/status", 10, true);
    dynamic_movement_primitive::ControllerStatusMsg dmp_status_msg;
    dmp_status_publisher_.initialize(publisher, 10, dmp_status_msg);

    return (initialized_ = true);
  }

// REAL-TIME REQUIREMENTS
template<class DMPType>
  void DMPDualArmControllerImplementation<DMPType>::publishStatus(const int status, const bool movement_finished, const ros::Time& end_time)
  {
    boost::shared_ptr<dynamic_movement_primitive::ControllerStatusMsg> status_msg;
    status_msg = dmp_status_publisher_.allocate();
    if (status_msg)
    {
      status_msg->status = status;
      status_msg->id = dmp_->getId();
      status_msg->percent_complete = dmp_->getProgress();
      status_msg->start_time = start_time_;
      if(movement_finished)
      {
        status_msg->end_time = end_time;
      }
      dmp_status_publisher_.publish(status_msg);
    }
  }

// REAL-TIME REQUIREMENTS
template<class DMPType>
  bool DMPDualArmControllerImplementation<DMPType>::newDMPReady()
  {

    return false;
  }

}

#endif /* DMP_DUAL_ARM_CONTROLLER_IMPLEMENTATION_H_ */

