/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         main-visualization.cpp

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#include <deep_learning/visualization.h>
#include <deep_learning/data_loader.h>
#include <deep_learning/data_grasp.h>
#include <deep_learning/extract_template.h>

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <iostream>

void test_0(ros::Publisher & p_pose) {

	std::cout << "test_0" << std::endl;
	geometry_msgs::PoseStamped result_view_point;
	result_view_point.header.frame_id = "BASE";
	result_view_point.header.stamp = ros::Time::now();

	p_pose.publish(result_view_point);
}

void test_1(ros::Publisher & p_pose) {

	std::cout << "test_1" << std::endl;
	geometry_msgs::PoseStamped result_view_point;
	result_view_point.header.frame_id = "BASE";
	result_view_point.header.stamp = ros::Time::now();
	result_view_point.pose.position.x = 1.0;
	tf::Quaternion t(tf::createQuaternionFromYaw(M_PI_2));
	result_view_point.pose.orientation.x = t.getX();
	result_view_point.pose.orientation.y = t.getY();
	result_view_point.pose.orientation.z = t.getZ();
	result_view_point.pose.orientation.w = t.getW();

	p_pose.publish(result_view_point);
}

void test_2(ros::Publisher & p_pose) {

	std::cout << "test_1" << std::endl;
	geometry_msgs::PoseStamped result_view_point;
	result_view_point.header.frame_id = "BASE";
	result_view_point.header.stamp = ros::Time::now();
	result_view_point.pose.position.x = 2.5;
	tf::Quaternion t(tf::createQuaternionFromYaw(M_PI));
	result_view_point.pose.orientation.x = t.getX();
	result_view_point.pose.orientation.y = t.getY();
	result_view_point.pose.orientation.z = t.getZ();
	result_view_point.pose.orientation.w = t.getW();

	p_pose.publish(result_view_point);
}

void test_3(ros::Publisher & p_pose) {

	std::cout << "test_3" << std::endl;
	geometry_msgs::PoseStamped result_view_point;
	result_view_point.header.frame_id = "BASE";
	result_view_point.header.stamp = ros::Time::now();
	result_view_point.pose.position.x = 1.0;
	tf::Quaternion t(tf::createQuaternionFromYaw(M_PI_2));
	result_view_point.pose.orientation.x = t.getX();
	result_view_point.pose.orientation.y = t.getY();
	result_view_point.pose.orientation.z = t.getZ();
	result_view_point.pose.orientation.w = t.getW();

	geometry_msgs::PoseStamped test;
	test.header.frame_id = "BASE";
	test.header.stamp = ros::Time::now();
	test.pose.position.x = 2.5;
	tf::Quaternion t2(tf::createQuaternionFromYaw(M_PI));
	test.pose.orientation.x = t2.getX();
	test.pose.orientation.y = t2.getY();
	test.pose.orientation.z = t2.getZ();
	test.pose.orientation.w = t2.getW();

	geometry_msgs::PoseStamped result;
	result.header.frame_id = "BASE";
	result.header.stamp = ros::Time::now();

	Extract_template::Coordinate_to_base(result_view_point.pose,test.pose,result.pose);
	std::cout << "base " << result_view_point.pose << std::endl;
	std::cout << "target " << test.pose << std::endl;
	std::cout << "result " << result.pose << std::endl;
	p_pose.publish(result);

}

void test_4(ros::Publisher & p_pose) {


	std::cout << "test_3" << std::endl;
	geometry_msgs::PoseStamped result_view_point;
	result_view_point.header.frame_id = "BASE";
	result_view_point.header.stamp = ros::Time::now();
	result_view_point.pose.position.x = 1.0;
	tf::Quaternion t(tf::createQuaternionFromYaw(M_PI_2));
	result_view_point.pose.orientation.x = t.getX();
	result_view_point.pose.orientation.y = t.getY();
	result_view_point.pose.orientation.z = t.getZ();
	result_view_point.pose.orientation.w = t.getW();

	geometry_msgs::PoseStamped test;
	test.header.frame_id = "BASE";
	test.header.stamp = ros::Time::now();
	test.pose.position.x = 2.5;
	tf::Quaternion t2(tf::createQuaternionFromYaw(M_PI));
	test.pose.orientation.x = t2.getX();
	test.pose.orientation.y = t2.getY();
	test.pose.orientation.z = t2.getZ();
	test.pose.orientation.w = t2.getW();

	geometry_msgs::PoseStamped result;
	result.header.frame_id = "BASE";
	result.header.stamp = ros::Time::now();

	Extract_template::Coordinate_to_base(result_view_point.pose,test.pose,result.pose);
	std::cout << "base " << result_view_point.pose << std::endl;
	std::cout << "target " << test.pose << std::endl;
	std::cout << "result " << result.pose << std::endl;

	Extract_template::Coordinate_to_world(result_view_point.pose,result.pose,test.pose);
	p_pose.publish(test);
}

int main(int argc, char** argv) {
	const std::string ROS_NH_NAME_SPACE = "test_coordinates";
	ros::init(argc, argv, ROS_NH_NAME_SPACE);
	ros::NodeHandle nh(ROS_NH_NAME_SPACE);
	ros::Publisher pose_0 = nh.advertise<geometry_msgs::PoseStamped>("pose_0",
			10);
	ros::Publisher pose_1 = nh.advertise<geometry_msgs::PoseStamped>("pose_1",
			10);
	ros::Publisher pose_2 = nh.advertise<geometry_msgs::PoseStamped>("pose_2",
			10);
	ros::Publisher pose_3 = nh.advertise<geometry_msgs::PoseStamped>("pose_3",
			10);
	ros::Publisher pose_4 = nh.advertise<geometry_msgs::PoseStamped>("pose_4",
			10);
	ros::Rate r(0.2);
	int counter = 0;
	geometry_msgs::Pose test_pose;
	test_pose.position.z = 0.5;
	while (ros::ok()) {
		counter += 1;

		std::cout << "counter " << counter << std::endl;
		test_0(pose_0);
		test_1(pose_1);
		test_2(pose_2);
		test_3(pose_3);
		test_4(pose_4);
		r.sleep();
	}

	return 0;
}
