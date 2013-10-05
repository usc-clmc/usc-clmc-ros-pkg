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
#include <Eigen/StdVector>
#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/linear_congruential.hpp>
#include <grasp_template/grasp_template.h>
#include <geometry_msgs/Pose.h>
#include <deep_learning/data_grasp.h>

namespace deep_learning {
//typedef boost::mt19937 base_generator_type;
typedef boost::minstd_rand base_generator_type;

class Extract_template {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d _bounding_box_corner_1;
	Eigen::Vector3d _bounding_box_corner_2;

	std::vector<grasp_template::GraspTemplate,
			Eigen::aligned_allocator<grasp_template::GraspTemplate> > _grasp_templates;

	Extract_template() :
			_bounding_box_corner_1(), _bounding_box_corner_2(), _grasp_templates(), _init_grasp_templates(
					false), _max_orientation_pertubation(0), _max_position_pertubation(
					0), _max_samples(0), _gripper_offsets(), generator(
					std::time(0)), uni_dist(-1, 1), _uni(generator, uni_dist) {

	}
	;
	Extract_template(Extract_template &extract_template) :
			_bounding_box_corner_1(extract_template._bounding_box_corner_1), _bounding_box_corner_2(
					extract_template._bounding_box_corner_2), _grasp_templates(
					extract_template._grasp_templates), _init_grasp_templates(
					false), _max_orientation_pertubation(0), _max_position_pertubation(
					0), _max_samples(0), _gripper_offsets(), generator(
					std::time(0)), uni_dist(-1, 1), _uni(generator, uni_dist) {

	}
	;

	Extract_template(Eigen::Vector3d bounding_box_corner_1,
			Eigen::Vector3d bounding_box_corner_2);
	virtual ~Extract_template() {
	}
	;

	static void Coordinate_to_world(const geometry_msgs::Pose &base_coordinate,
			const geometry_msgs::Pose &target_coordinate,
			geometry_msgs::Pose &result_coordinate);

	static void Coordinate_to_base(const geometry_msgs::Pose &base_coordinate,
			const geometry_msgs::Pose &target_coordinate,
			geometry_msgs::Pose &result_coordinate);

	void Init_grasp_templates(std::vector<Data_grasp> &pgrasp_templates);

	inline	std::vector<grasp_template::GraspTemplate,
					Eigen::aligned_allocator<grasp_template::GraspTemplate> >& Get_grasp_templates() {
		assert(_init_grasp_templates);
		return _grasp_templates;
	}

	inline std::vector<std::size_t> & Get_uuids_data_grasp(){
		assert(_init_grasp_templates);
		return _uuids_data_grasp;
	}
	;

	void Get_random_grasp_templates(grasp_template::GraspTemplate &g_temp,
			std::vector<grasp_template::GraspTemplate,
					Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
			std::vector<geometry_msgs::Pose> &result_gripper_pose);

	void Get_random_grasp_templates(grasp_template::GraspTemplate &g_temp,
			std::vector<grasp_template::GraspTemplate,
					Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
			std::vector<geometry_msgs::Pose> &result_gripper_pose,
			geometry_msgs::Pose &gripper_pose_offset);

	void Get_normal_grasp_templates(grasp_template::GraspTemplate &g_temp,
			std::vector<grasp_template::GraspTemplate,
					Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
			std::vector<geometry_msgs::Pose> &result_gripper_pose);

	void Get_normal_grasp_templates(grasp_template::GraspTemplate &g_temp,
			std::vector<grasp_template::GraspTemplate,
					Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
			std::vector<geometry_msgs::Pose> &result_gripper_pose,
			geometry_msgs::Pose &gripper_pose_offset);

private:

	bool _init_grasp_templates;
	double _max_orientation_pertubation;
	double _max_position_pertubation;
	int _max_samples;
	std::vector<geometry_msgs::Pose> _gripper_offsets;
	std::vector<std::size_t> _uuids_data_grasp;

	base_generator_type generator;
	boost::uniform_real<float> uni_dist;
	boost::variate_generator<base_generator_type&, boost::uniform_real<float> > _uni;

	double _Get_sample_value(double scaling);
	Eigen::Quaternion<double> _Get_sample_orientation(
			Eigen::Quaternion<double> &base_orientation);

	Eigen::Quaternion<double> _Get_sample_orientation_z(
			Eigen::Quaternion<double> &base_orientation);
protected:
};
}

#endif /*EXTRACT_TEMPLATE_H*/
