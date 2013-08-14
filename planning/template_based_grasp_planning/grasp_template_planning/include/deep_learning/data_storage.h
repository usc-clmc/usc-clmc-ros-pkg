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

#include <deep_learning/data_grasp.h>
#include <deep_learning/def.h>

#include <map>

namespace fs = boost::filesystem3;

namespace deep_learning {
class Data_storage {
private:
	boost::filesystem::path _path_dir;
	YAML::Emitter _doc;
	rosbag::Bag _database;
	boost::filesystem::path _file_path_dataset;
	boost::filesystem::path _file_path_database;


protected:
	// yaml::node is not copyable
	Data_storage(Data_storage &data_storage) {
	}
	;
public:

	Data_storage(const std::string& path);
	virtual ~Data_storage() {
		_database.close();
	}
	;

	void Init_dataset(const std::string& dataset_name);
	void Update_dataset(grasp_template::TemplateHeightmap &heightmap);
	void Update_dataset(grasp_template::TemplateHeightmap &heightmap,
			const std::string &grasp_uid, float grasp_success);
	void Store_dataset();

	void Init_database(const std::string &database_name);
	void Store_database(std::map<std::string, Data_grasp> &result_grasps);

	inline boost::filesystem::path Get_file_path_dataset(){
		return _file_path_database;
	}
	inline boost::filesystem::path Get_file_path_database(){
		return _file_path_database;
	}
};
}

#endif /*DATA_STORAGE_H*/
