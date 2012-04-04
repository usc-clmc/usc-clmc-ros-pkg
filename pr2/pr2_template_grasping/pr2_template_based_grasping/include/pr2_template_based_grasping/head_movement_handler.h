/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         head_movment_handler.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef HEAD_MOVEMENT_HANDLER_H_
#define HEAD_MOVEMENT_HANDLER_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

namespace pr2_template_based_grasping
{
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class HeadMovementHandler
{
public:
  HeadMovementHandler();
  ~HeadMovementHandler();

  //! Points the camera frame at a point in a given frame
  void lookAt(std::string frame_id, double x, double y, double z);

private:
  PointHeadClient* point_head_client_;
};

} //namespace
#endif /* HEAD_MOVEMENT_HANDLER_H_ */
