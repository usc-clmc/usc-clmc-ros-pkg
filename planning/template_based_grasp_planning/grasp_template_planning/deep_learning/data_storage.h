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

#include <opencv2/core/core.hpp>
#include <grasp_template/template_heightmap.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem3;

class Data_storage {
private:
	boost::filesystem::path _path_dir;
	YAML::Emitter _doc;

protected:
	// yaml::node is not copyable
	Data_storage(Data_storage &data_storage){};
public:
	Data_storage(const std::string& path);
	virtual ~Data_storage(){};

	cv::Mat& Render_image(grasp_template::TemplateHeightmap &heightmap);

	bool Store(grasp_template::TemplateHeightmap &heightmap);
	bool Store(grasp_template::TemplateHeightmap &heightmap,
			const std::string &grasp_uid, float grasp_success);

	void Init_meta_data();
};

#endif /*DATA_STORAGE_H*/
