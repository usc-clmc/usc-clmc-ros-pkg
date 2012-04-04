/*
 * palm_marker.cpp
 *
 *  Created on: Feb 20, 2011
 *      Author: herzog
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace visualization_msgs;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "palm_marker");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<Marker>("palm_marker", 100);

	while(ros::ok())
	{
		Marker m;

		m.header.frame_id = "/r_gripper_led_frame";
		m.header.stamp = ros::Time::now();

		m.ns = "palm_marker";
		m.id = 1;

		m.type = Marker::POINTS;
		m.action = Marker::ADD;

		m.pose.orientation.w = 1.0;
		m.scale.x = 0.01;
		m.scale.y = 0.01;
		m.color.r = 1.0;
		m.color.a = 1.0;

		geometry_msgs::Point p;
		p.x = 0.08;
		p.y = 0.0;
		p.z = -0.025;

		m.points.push_back(p);

		marker_pub.publish(m);

		r.sleep();
	}
}
