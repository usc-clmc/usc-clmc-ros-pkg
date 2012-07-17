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

InteractiveCandidateFilter::InteractiveCandidateFilter(ros::NodeHandle& n): nh_(n), int_marker_server("exclusion_ball_server")
{
	ball_max_scale_ = 0.3;
//  ros::init(argc, argv, "simple_marker");

  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "exclusion_ball_marker";
  int_marker.description = "Grasp Point Exclusion";
  int_marker.scale = ball_max_scale_;
  excluding_ball_diameter_ = 0.1;

  ball_pub_ = nh_.advertise<visualization_msgs::Marker>("ghm_exclusion_ball", 5);

  // create a grey ball marker
  ball_marker_.header.frame_id = "/base_link";
  ball_marker_.header.stamp = ros::Time::now();
  ball_marker_.type = Marker::SPHERE;
  ball_marker_.scale.x = excluding_ball_diameter_;
  ball_marker_.scale.y = excluding_ball_diameter_;
  ball_marker_.scale.z = excluding_ball_diameter_;
  ball_marker_.color.r = 0.5;
  ball_marker_.color.g = 0.5;
  ball_marker_.color.b = 0.5;
  ball_marker_.color.a = 0.8;
  ball_pub_.publish(ball_marker_);


  // create a non-interactive control which contains the box
  InteractiveMarkerControl ball_control;
  ball_control.name = "ball";
  ball_control.always_visible = true;
//  ball_control.markers.push_back( ball_marker_ );

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

	excluding_ball_diameter_ = feedback->pose.orientation.w * ball_max_scale_;
//	std::cout << excluding_ball_diameter_ << std::endl;

	ball_marker_.header.stamp = ros::Time::now();
	ball_marker_.pose.position = feedback->pose.position;
	ball_marker_.scale.x = excluding_ball_diameter_;
	ball_marker_.scale.y = excluding_ball_diameter_;
	ball_marker_.scale.z = excluding_ball_diameter_;
	ball_pub_.publish(ball_marker_);
}

bool InteractiveCandidateFilter::isGraspFiltered(const geometry_msgs::Point& template_origin) const
{
	double dx = template_origin.x - exclusion_ball_center_.x;
	double dy = template_origin.y - exclusion_ball_center_.y;
	double dz = template_origin.z - exclusion_ball_center_.z;

	return (dx*dx + dy*dy + dz*dz) < 0.25 * excluding_ball_diameter_ * excluding_ball_diameter_;
}

}  //namespace
