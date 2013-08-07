/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         extract_template.cpp

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#include <deep_learning/extract_template.h>

#include <ctime>
#include <grasp_template/dismatch_measure.h>

Extract_template::Extract_template(Eigen::Vector3d bounding_box_corner_1,
		Eigen::Vector3d bounding_box_corner_2,
		geometry_msgs::Pose &gripper_pose) :
		_bounding_box_corner_1(bounding_box_corner_1), _bounding_box_corner_2(
				bounding_box_corner_2), _gripper_pose(gripper_pose), generator(
				std::time(0)), uni_dist(-1, 1), _uni(generator, uni_dist) {
	// have to rescale the random stuff
	_max_position_pertubation = 0.1;
	_max_orientation_pertubation = 0.1;
	_max_samples = 10;
}

void Extract_template::Coordinate_transformation(
		geometry_msgs::Pose &base_coordinate,
		geometry_msgs::Pose &target_coordinate,
		geometry_msgs::Pose &result_coordinate) {

	Eigen::Vector3d template_translation;
	template_translation.x() = base_coordinate.position.x;
	template_translation.y() = base_coordinate.position.y;
	template_translation.z() = base_coordinate.position.z;

	Eigen::Quaterniond template_orientation;
	template_orientation.x() = base_coordinate.orientation.x;
	template_orientation.y() = base_coordinate.orientation.y;
	template_orientation.z() = base_coordinate.orientation.z;
	template_orientation.w() = base_coordinate.orientation.w;

	Eigen::Vector3d gripper_translation;
	gripper_translation.x() = target_coordinate.position.x;
	gripper_translation.y() = target_coordinate.position.y;
	gripper_translation.z() = target_coordinate.position.z;

	Eigen::Quaterniond gripper_orientation;
	gripper_orientation.x() = target_coordinate.orientation.x;
	gripper_orientation.y() = target_coordinate.orientation.y;
	gripper_orientation.z() = target_coordinate.orientation.z;
	gripper_orientation.w() = target_coordinate.orientation.w;


	Eigen::Quaterniond result_orientation (template_orientation.toRotationMatrix().transpose() * gripper_orientation.toRotationMatrix());
	Eigen::Vector3d result_translation = template_orientation.toRotationMatrix().transpose()*gripper_translation-template_translation;

	result_coordinate.position.x = result_translation.x();
	result_coordinate.position.y = result_translation.y();
	result_coordinate.position.z = result_translation.z();

	result_coordinate.orientation.x = result_orientation.x();
	result_coordinate.orientation.y = result_orientation.y();
	result_coordinate.orientation.z = result_orientation.z();
	result_coordinate.orientation.w = result_orientation.w();
}

void Extract_template::Get_random_grasp_templates(
		grasp_template::GraspTemplate &g_temp,
		std::vector<grasp_template::GraspTemplate,
				Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template) {
	result_template.clear();

	geometry_msgs::Pose original_pose = _gripper_pose;
	//g_temp.getPose(original_pose);
	geometry_msgs::Pose identity_pose;
	//g_temp.getPose(identity_pose);
	geometry_msgs::Pose gripper_pose = _gripper_pose;

	std::cout << "original_pose " << original_pose << std::endl;
	std::cout << "identity_pose " << identity_pose << std::endl;
	std::cout << "gripper_pose " << gripper_pose << std::endl;
	for (int i = 0; i < _max_samples; ++i) {
		grasp_template::GraspTemplate local_grasp_template(g_temp);

		// sample templt_pose
		// this should be identity for now

		// sample gripper_pose
		// transform this in the template frame and then
		// sample from the applied grasps
		// this is required to actually compute the mask

		// sample x,y,z
		gripper_pose.position.x = original_pose.position.x
				+ _Get_sample_value(_max_position_pertubation);
		gripper_pose.position.y = original_pose.position.y
				+ _Get_sample_value(_max_position_pertubation);
		gripper_pose.position.z = original_pose.position.z
				+ _Get_sample_value(_max_position_pertubation);

		std::cout << "sampled gripper_pose " << gripper_pose << std::endl;

		Eigen::Quaternion<double> original_orientation;
		original_orientation.x() = original_pose.orientation.x;
		original_orientation.y() = original_pose.orientation.y;
		original_orientation.z() = original_pose.orientation.z;
		original_orientation.w() = original_pose.orientation.w;
		Eigen::Quaternion<double> sampled_orientation = _Get_sample_orientation(
				original_orientation);

		//std::cout << "sampled orientation " << sampled_orientation << std::endl;
		gripper_pose.orientation.x = sampled_orientation.x();
		gripper_pose.orientation.y = sampled_orientation.y();
		gripper_pose.orientation.z = sampled_orientation.z();
		gripper_pose.orientation.w = sampled_orientation.w();

		//std::cout << "sampled gripper_pose " << gripper_pose << std::endl;

		// heightmap is not important
		grasp_template::Heightmap tmp_heightmap;
		g_temp.heightmap_.toHeightmapMsg(tmp_heightmap);
		grasp_template::DismatchMeasure d_measure(tmp_heightmap, identity_pose,
				gripper_pose, _bounding_box_corner_1, _bounding_box_corner_2);
		d_measure.applyDcMask(local_grasp_template);
		result_template.push_back(local_grasp_template);
	}
}
double Extract_template::_Get_sample_value(double scaling) {
	return _uni() * scaling;
}

Eigen::Quaternion<double> Extract_template::_Get_sample_orientation(
		Eigen::Quaternion<double> &base_orientation) {
	// sample roll,pitch,yaw
	// I can use roll pitch yaw since I will only sample small rotations
	// such that a gimbal lock will most likely not occur

	Eigen::Quaternion<double> x_rotation(
			Eigen::AngleAxisd(_Get_sample_value(_max_orientation_pertubation),
					Eigen::Vector3d::UnitZ()));
	Eigen::Quaternion<double> y_rotation(
			Eigen::AngleAxisd(_Get_sample_value(_max_orientation_pertubation),
					Eigen::Vector3d::UnitY()));
	Eigen::Quaternion<double> z_rotation(
			Eigen::AngleAxisd(_Get_sample_value(_max_orientation_pertubation),
					Eigen::Vector3d::UnitZ()));
	return base_orientation * x_rotation * y_rotation * z_rotation;
}
