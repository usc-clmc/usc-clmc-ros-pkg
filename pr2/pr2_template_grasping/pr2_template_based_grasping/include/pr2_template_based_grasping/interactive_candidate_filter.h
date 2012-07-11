/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         interactive_candidate_filter.h

 \author       Alexander Herzog
 \date         July 9, 2012

 *********************************************************************/

#ifndef INTERACTIVE_CANDIDATE_FILTER_
#define INTERACTIVE_CANDIDATE_FILTER_

#include <geometry_msgs/Point.h>
#include <interactive_markers/interactive_marker_server.h>

namespace pr2_template_based_grasping
{

class InteractiveCandidateFilter
{
public:
	InteractiveCandidateFilter(double radius = 0.0);
	bool isGraspFiltered(const geometry_msgs::Point& template_origin) const;

	void processFeedback(
	    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

private:
	geometry_msgs::Point exclusion_ball_center_;
	double excluding_ball_radius_;
	interactive_markers::InteractiveMarkerServer int_marker_server;
};

}  //namespace
#endif  /* INTERACTIVE_CANDIDATE_FILTER_ */
