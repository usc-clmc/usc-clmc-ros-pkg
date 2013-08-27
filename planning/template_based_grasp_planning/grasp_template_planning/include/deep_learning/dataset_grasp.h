/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         dataset_grasp.h

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#ifndef DATASET_GRASP_H
#define DATASET_GRASP_H

#include <string>
#include <grasp_template/grasp_template.h>
#include <geometry_msgs/Pose.h>
#include <deep_learning/def.h>
namespace deep_learning {

class Dataset_grasp {
private:
public:
	std::string file_name_bag;
	grasp_template::GraspTemplate grasp_template;
	geometry_msgs::Pose gripper_pose;
	std::string uuid_dataset;
	std::size_t uuid_database;
	float success;
	std::vector<std::size_t> uuids_database;
	std::vector<double> scores;

	Dataset_grasp() :
			file_name_bag(), grasp_template(), gripper_pose(), uuid_dataset(), success(), uuids_database(), scores() {
	}
	;
	Dataset_grasp(const std::string &pfile_name_bag,
			const grasp_template::GraspTemplate &pgrasp_template,
			const geometry_msgs::Pose pgripper_pose,
			const std::vector<std::size_t> &puuids_data_grasp,
			const std::vector<double> &pscores) :
			file_name_bag(pfile_name_bag), grasp_template(pgrasp_template), gripper_pose(
					pgripper_pose), uuid_dataset(UUID_NONE), uuid_database(0), success(
					SUCCESS_FALSE), uuids_database(puuids_data_grasp), scores(
					pscores) {
	}
	;

	Dataset_grasp(const std::string &pfile_name_bag,
			const grasp_template::GraspTemplate &pgrasp_template,
			const geometry_msgs::Pose pgripper_pose,
			std::size_t puuid_database_grasp, float psuccess) :
			file_name_bag(pfile_name_bag), grasp_template(pgrasp_template), gripper_pose(
					pgripper_pose), uuid_dataset(UUID_NONE), uuid_database(
					puuid_database_grasp), success(psuccess), uuids_database(), scores() {
	}
	;

	virtual ~Dataset_grasp() {
	}
	;
};
}
#endif /*DATASET_GRASP_H*/
