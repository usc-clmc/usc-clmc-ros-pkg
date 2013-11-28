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

#include <deep_learning/extract_template.h>
#include <deep_learning/database_grasp.h>
#include <deep_learning/data_grasp.h>
#include <deep_learning/dataset_grasp.h>
#include <deep_learning/def.h>

#include <io_hdf5/io_hdf5.h>

#include <map>

namespace fs = boost::filesystem3;

namespace deep_learning {
class Data_storage {
private:
	YAML::Emitter _doc;
	rosbag::Bag _database;
	boost::filesystem::path _file_path_dataset;
	boost::filesystem::path _file_path_database;
	boost::filesystem::path _dir_path_dataset;
	rosbag::Bag _grasp_dataset;

	io_hdf5::IO_hdf5Ptr _io_hdf5;

	Extract_template *_pextract_template;


protected:
	// yaml::node is not copyable
	Data_storage(Data_storage &data_storage) {
	}
	;
public:

	Data_storage(){

	}

	virtual ~Data_storage() {
		_database.close();
	}
	;

	void Init_dataset(const std::string& path,const std::string& dataset_name, Extract_template *pextract_template);
	void Update_dataset(const Dataset_grasp &dataset_grasp);
	void Update_dataset(std::map<std::size_t, Dataset_grasp> &result_grasps);
	void Store_dataset();
	void Init_hdf5(io_hdf5::IO_hdf5Ptr io_hdf5);
	void Update_hdf5(std::vector<Dataset_grasp> &result_grasps);

	void Init_database(const std::string& path,const std::string &database_name);
	void Store_database(Database_grasp &database_grasp);

	inline boost::filesystem::path Get_file_path_dataset(){
		return _file_path_database;
	}
	inline boost::filesystem::path Get_file_path_database(){
		return _file_path_database;
	}
};
}

#endif /*DATA_STORAGE_H*/
