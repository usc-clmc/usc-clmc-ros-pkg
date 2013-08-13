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

#include <opencv2/core/core.hpp>
#include <grasp_template/grasp_template.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <deep_learning/data_grasp.h>

namespace deep_learning {
class Data_loader {
private:
	std::string _log_topic;
public:
	Data_loader(const std::string &log_topic);
	virtual ~Data_loader() {
	}
	;

	bool Load_trial_log(const std::string &path_bagfile,
			std::vector<Data_grasp> &result_grasp,
			sensor_msgs::PointCloud2 &result_object_cloud,
			geometry_msgs::Pose &result_view_point);

	Data_grasp Load_grasp_template(const std::string &path_bagfile);

	bool Load_grasp_database(const std::string &path_bagfile,std::vector<Data_grasp> &result_grasps);

};
}

#endif /*DATA_STORAGE_H*/
