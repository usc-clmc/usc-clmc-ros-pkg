/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         extract_template.h

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#ifndef EXTRACT_TEMPLATE_H
#define EXTRACT_TEMPLATE_H

#include <Eigen/Eigen>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/linear_congruential.hpp>
#include <grasp_template/grasp_template.h>
#include <geometry_msgs/Pose.h>

namespace deep_learning {
//typedef boost::mt19937 base_generator_type;
typedef boost::minstd_rand base_generator_type;

class Extract_template {
private:

	Eigen::Vector3d _bounding_box_corner_1;
	Eigen::Vector3d _bounding_box_corner_2;
	geometry_msgs::Pose _gripper_pose;
	double _max_orientation_pertubation;
	double _max_position_pertubation;
	int _max_samples;

	base_generator_type generator;
	boost::uniform_real<float> uni_dist;
	boost::variate_generator<base_generator_type&, boost::uniform_real<float> > _uni;

	Extract_template() :
			generator(std::time(0)), uni_dist(-1, 1), _uni(generator, uni_dist) {
	}
	;
	Extract_template(Extract_template &extract_template) :
			generator(std::time(0)), uni_dist(-1, 1), _uni(generator, uni_dist) {
	}
	;

	double _Get_sample_value(double scaling);
	Eigen::Quaternion<double> _Get_sample_orientation(
			Eigen::Quaternion<double> &base_orientation);

protected:
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Extract_template(Eigen::Vector3d bounding_box_corner_1,
			Eigen::Vector3d bounding_box_corner_2,
			geometry_msgs::Pose &gripper_pose);
	virtual ~Extract_template() {
	}
	;

	static void Coordinate_to_world(geometry_msgs::Pose &base_coordinate,
			geometry_msgs::Pose &target_coordinate,
			geometry_msgs::Pose &result_coordinate);

	static void Coordinate_to_base(geometry_msgs::Pose &base_coordinate,
			geometry_msgs::Pose &target_coordinate,
			geometry_msgs::Pose &result_coordinate);

	void Get_random_grasp_templates(grasp_template::GraspTemplate &g_temp,
			std::vector<grasp_template::GraspTemplate,
					Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
			std::vector<geometry_msgs::Pose> &result_gripper_pose);

};
}

#endif /*EXTRACT_TEMPLATE_H*/
