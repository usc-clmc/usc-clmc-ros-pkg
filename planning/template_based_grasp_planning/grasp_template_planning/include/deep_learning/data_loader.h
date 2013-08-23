/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         log_loader.h

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <grasp_template/grasp_template.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <deep_learning/data_grasp.h>
#include <deep_learning/dataset_grasp.h>
#include <deep_learning/extract_template.h>

namespace deep_learning {
class Data_loader {
private:
	Extract_template *_pextract_template;
	std::vector<Data_grasp> _grasp_templates;
public:
	Data_loader();
	virtual ~Data_loader() {
		if (_pextract_template != NULL) {
			delete _pextract_template;
		}
	}
	;

	void Init_dataset(std::vector<Data_grasp> &grasp_templates);
	bool Extract_dataset(const std::string &path_bagfile,
			const std::string &topic, std::vector<Dataset_grasp> &result_grasp);

	Data_grasp Load_grasp_template(const std::string &path_bagfile,
			const std::string &topic);

	bool Load_grasp_database(const std::string &path_bagfile,
			const std::string &topic, std::vector<Data_grasp> &result_grasps);

};
}

#endif /*DATA_STORAGE_H*/
