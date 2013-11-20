/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         dataset_grasp.cpp

 \author       Daniel Kappler
 \date         July 31, 2013

 *********************************************************************/

#include <deep_learning/dataset_grasp.h>

#include <deep_learning/def.h>
#include <deep_learning/utils.h>
#include <grasp_template/dismatch_measure.h>

namespace deep_learning {

void Dataset_grasp::Add(const std::string &path_bagfile,
		grasp_template::GraspTemplate &grasp_template,
		Extract_template &extract_template,
		std::vector<Dataset_grasp> &result_dataset_grasps) {

	std::cout << "add " << path_bagfile << std::endl;
	std::vector<std::size_t> uuids_data_grasp =
			extract_template.Get_uuids_data_grasp();
	std::vector<double> scores;
	scores.resize(uuids_data_grasp.size());
	grasp_template::DismatchMeasure dissmatch_measure;
	grasp_template::TemplateDissimilarity score;
	std::vector<geometry_msgs::Pose> gripper_pose;
	std::vector<grasp_template::GraspTemplate,
			Eigen::aligned_allocator<grasp_template::GraspTemplate> > random_templates;

	std::vector<grasp_template::GraspTemplate,
			Eigen::aligned_allocator<grasp_template::GraspTemplate> > grasp_templates;
	grasp_templates.push_back( extract_template.Get_grasp_templates()[0]);

	//extract_template.Get_random_grasp_templates(grasp_template,
			//random_templates, gripper_pose);


	extract_template.Get_normal_grasp_templates(grasp_template,
			random_templates, gripper_pose);

	for (unsigned int j = 0; j < random_templates.size(); ++j) {
		// score
		/*
		for (unsigned int k = 0; k < grasp_templates.size(); ++k) {
			score = dissmatch_measure.getScore(grasp_templates[k],
					random_templates[j]);
			scores[k] = score.getScore();
		}
		*/

		result_dataset_grasps.push_back(
				Dataset_grasp(path_bagfile, random_templates[j],
						gripper_pose[j], uuids_data_grasp, scores));
	}

}
void Dataset_grasp::Add(Data_grasp &data_grasp,
		std::vector<Dataset_grasp> &result_dataset_grasps) {

	grasp_template::GraspTemplate grasp_template = data_grasp.grasp_template;

	Dataset_grasp grasp;
	grasp.file_name_bag = DATA_GRASP_PATH_BAGFILE;
	grasp.grasp_template = data_grasp.grasp_template;
	grasp.gripper_pose = data_grasp.gripper_pose;
	grasp.uuid_original = data_grasp.uuid_original;
	grasp.uuid_database = data_grasp.uuid_database;
	grasp.uuid_database_template = data_grasp.uuid_database_template;
	grasp.success = data_grasp.success;
	result_dataset_grasps.push_back(grasp);

}
}
