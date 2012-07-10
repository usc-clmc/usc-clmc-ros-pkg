/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         interactive_candidate_filter.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/
#include <pr2_template_based_grasping/interactive_candidate_filter.h>

using namespace visualization_msgs;

namespace pr2_template_based_grasping
{

InteractiveCandidateFilter::InteractiveCandidateFilter(double radius): excluding_ball_radius_(radius), int_marker_server("exclusion_ball_server")
{
//  ros::init(argc, argv, "simple_marker");

  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "exclusion_ball_marker";
  int_marker.description = "Simple 4-DOF Control";

  // create a grey ball marker
  Marker ball_marker;
  ball_marker.type = Marker::SPHERE;
  ball_marker.scale.x = 0.10;
  ball_marker.scale.y = 0.10;
  ball_marker.scale.z = 0.10;
  ball_marker.color.r = 0.5;
  ball_marker.color.g = 0.5;
  ball_marker.color.b = 0.5;
  ball_marker.color.a = 0.8;

  // create a non-interactive control which contains the box
  InteractiveMarkerControl ball_control;
  ball_control.name = "ball";
  ball_control.always_visible = true;
  ball_control.markers.push_back( ball_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( ball_control );

	InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
//	control.name = "rotate_x";
//	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//	int_marker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
//	control.name = "rotate_y";
//	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//	int_marker.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  int_marker_server.insert(int_marker,  boost::bind(&InteractiveCandidateFilter::processFeedback, this, _1));

  // 'commit' changes and send to all clients
  int_marker_server.applyChanges();
};

void InteractiveCandidateFilter::processFeedback(
	    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	exclusion_ball_center_ = feedback->pose.position;
}

bool InteractiveCandidateFilter::isGraspFiltered(const geometry_msgs::Point& template_origin)
{
	double dx = template_origin.x - exclusion_ball_center_.x;
	double dy = template_origin.y - exclusion_ball_center_.y;
	double dz = template_origin.z - exclusion_ball_center_.z;

	return (dx*dx + dy*dy + dz*dz) < excluding_ball_radius_ * excluding_ball_radius_;
}

}  //namespace
