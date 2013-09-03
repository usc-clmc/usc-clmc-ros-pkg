/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         data_grasp.cpp

 \author       Daniel Kappler
 \date         July 31, 2013

 *********************************************************************/

#include <deep_learning/data_grasp.h>

#include <deep_learning/def.h>
#include <deep_learning/utils.h>

namespace deep_learning {

Data_grasp::Data_grasp(const std::vector<double> &pgripper_joints,
		const geometry_msgs::Pose &pgripper_pose,
		const grasp_template::GraspTemplate &pgrasp_template,
		const std::size_t &puuid_database,
		const std::size_t &puuid_database_template, int psuccess) :
		gripper_joints(pgripper_joints), gripper_pose(pgripper_pose), grasp_template(
				pgrasp_template), uuid_database(puuid_database), uuid_database_template(
				puuid_database_template), success(psuccess) {
}
Data_grasp::Data_grasp(const std::vector<double> &pgripper_joints,
		const geometry_msgs::Pose &pgripper_pose,
		const grasp_template::GraspTemplate &pgrasp_template) :
		gripper_joints(pgripper_joints), gripper_pose(pgripper_pose), grasp_template(
				pgrasp_template), uuid_database(0), uuid_database_template(0), success(
				SUCCESS_FALSE) {
}

Data_grasp::Data_grasp(const std::vector<double> &pgripper_joints,
		const geometry_msgs::Pose &pgripper_pose,
		const grasp_template::GraspTemplate &pgrasp_template,int psuccess) :
		gripper_joints(pgripper_joints), gripper_pose(pgripper_pose), grasp_template(
				pgrasp_template), uuid_database(0), uuid_database_template(0), success(
				psuccess) {
}

Data_grasp::Data_grasp() :
		gripper_pose(), grasp_template(), uuid_database(0),uuid_database_template(0), success(
				SUCCESS_FALSE) {

}

Data_grasp::Data_grasp(Data_grasp_log &d_grasp) :
		gripper_joints(d_grasp.gripper_joints.vals), gripper_pose(
				d_grasp.gripper_pose.pose), grasp_template(
				d_grasp.grasp_template_heightmap,
				d_grasp.grasp_template_pose.pose), uuid_database(
				d_grasp.uuid_database), uuid_database_template(
				d_grasp.uuid_database_template), success(d_grasp.success) {
}

void Data_grasp::Valid_uuid() {
	if (uuid_database == 0) {
		boost::hash<grasp_database_hash> data_grasp_hasher;
		uuid_database = data_grasp_hasher(*this);
	}
	if (uuid_database_template == 0) {
		boost::hash<grasp_database_template_hash> data_grasp_template_hasher;
		uuid_database_template = data_grasp_template_hasher(*this);
	}
}


}
