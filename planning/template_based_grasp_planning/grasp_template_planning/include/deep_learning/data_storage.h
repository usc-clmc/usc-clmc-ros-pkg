/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         data_storage.h

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#ifndef DATASTORAGE_H
#define DATASTORAGE_H

#include <rosbag/bag.h>
#include <opencv2/core/core.hpp>
#include <grasp_template/template_heightmap.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include "data_grasp.h"

#include <map>

namespace fs = boost::filesystem3;

namespace deep_learning {
class Data_storage {
private:
	boost::filesystem::path _path_dir;
	YAML::Emitter _doc;
	rosbag::Bag _database;

protected:
	// yaml::node is not copyable
	Data_storage(Data_storage &data_storage) {
	}
	;
public:
	boost::filesystem::path file_path_meta;
	boost::filesystem::path file_path_database;

	Data_storage(const std::string& path);
	virtual ~Data_storage() {
		_database.close();
	}
	;

	cv::Mat Render_image(grasp_template::TemplateHeightmap &heightmap);

	bool Store(grasp_template::TemplateHeightmap &heightmap);
	bool Store(grasp_template::TemplateHeightmap &heightmap,
			const std::string &grasp_uid, float grasp_success);

	bool Store_grasp_database(std::map<std::string, Data_grasp> &result_grasps);

	void Store_meta();

	void Init_meta_data();
	void Init_database();

};
}

#endif /*DATA_STORAGE_H*/
