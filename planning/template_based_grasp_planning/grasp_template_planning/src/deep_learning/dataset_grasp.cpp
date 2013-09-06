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

namespace deep_learning {

void Dataset_grasp::Add(
		const grasp_template::GraspTemplate &grasp_template,
		const Extract_template &extract_template,
		std::vector<Dataset_grasp> &result_dataset_grasps) {

}
void Dataset_grasp::Add(const Data_grasp &data_grasp,
		const Extract_template &extract_template,
		std::vector<Dataset_grasp> &result_dataset_grasps) {

}
}
