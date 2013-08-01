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

#ifndef LOG_LOADER_H
#define LOG_LOADER_H

#include <opencv2/core/core.hpp>
#include <grasp_template/grasp_template.h>

class Log_loader {
private:
	std::string _log_topic;
public:
	Log_loader(const std::string &log_topic);
	virtual ~Log_loader() {
	}
	;

	bool Load_trial_log(const std::string &path_bagfile,
			std::vector<grasp_template::GraspTemplate,
					Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
			std::vector<std::string> &result_uuid,
			std::vector<float> &result_success);

};

#endif /*DATA_STORAGE_H*/
