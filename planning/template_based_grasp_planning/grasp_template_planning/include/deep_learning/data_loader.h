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
#include <deep_learning/database_grasp.h>
#include <deep_learning/dataset_grasp.h>
#include <deep_learning/extract_template.h>

// debugging
#include <deep_learning/visualization.h>
// debugging

namespace deep_learning {
class Data_loader {
public:
	Data_loader();
	virtual ~Data_loader() {
	}
	;

	bool Load_from_grasp_log(const std::string &path_bagfile,
			const std::string &topic,
			Extract_template &extract_template,
			std::vector<Dataset_grasp> &result_dataset_grasps);

	bool Load_from_library(const std::string &path_bagfile,
			std::vector<Data_grasp> &result_dataset_grasps,int success);

	Database_grasp Load_grasp_database(const std::string &path_bagfile);

// debugging
	sensor_msgs::PointCloud2 _point_cloud;
// debugging
};
}

#endif /*DATA_STORAGE_H*/
