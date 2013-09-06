/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         data_storage.cpp

 \author       Daniel Kappler
 \date         July 31, 2013

 *********************************************************************/

#include <deep_learning/database_grasp.h>

#include <deep_learning/def.h>

namespace deep_learning {

Database_grasp::Database_grasp():_grasp_templates_success(),_grasp_templates_fail(){
}
Database_grasp::~Database_grasp(){

}

void Database_grasp::Add_grasp(Data_grasp &grasp_template){
	grasp_template.Valid_uuid();
	//std::cout << "uuid_database " << grasp_template.uuid_database << std::endl;
	//std::cout << "uuid_database_templates " << grasp_template.uuid_database_template << std::endl;
	//std::cout << "grasp_template.success " << grasp_template.success << std::endl;
	if(grasp_template.success == SUCCESS_TRUE){
		//std::cout << "SUCCESS" << std::endl;
		_Add_grasp(grasp_template,_grasp_templates_success);
		return;
	}
	if(grasp_template.success == SUCCESS_FALSE){
		//std::cout << "FAIL" << std::endl;
		_Add_grasp(grasp_template,_grasp_templates_fail);
		return;
	}
	ROS_ERROR("grasp_template is neither success nor fail");
}

void Database_grasp::_Add_grasp(Data_grasp &grasp_template,database_type &grasp_templates){
	std::vector<Data_grasp> *tmp_grasp_templates = &grasp_templates[grasp_template.uuid_database];
	// make sure that we have every template only once
	for(unsigned int i = 0; i < tmp_grasp_templates->size(); ++i){
		if ((*tmp_grasp_templates)[i].uuid_database_template == grasp_template.uuid_database_template){
			std::cout << "ALREADY IN DATABASE" << std::endl;
			std::cout << "uuid_database " << grasp_template.uuid_database << std::endl;
			std::cout << "uuid_database_templates " << grasp_template.uuid_database_template << std::endl;
			return;
		}
	}
	tmp_grasp_templates->push_back(grasp_template);
}

Data_grasp Database_grasp::Get_grasp(const size_t uuid_database,const size_t uuid_database_template){
	Data_grasp tmp = _Get_grasp(uuid_database,uuid_database_template,_grasp_templates_success);
	if (tmp.uuid_database == 0){
		return _Get_grasp(uuid_database,uuid_database_template,_grasp_templates_fail);
	}
	return tmp;
}

Data_grasp Database_grasp::_Get_grasp(const size_t uuid_database,const size_t uuid_database_template,database_type &grasp_templates){
	std::vector<Data_grasp> *tmp_grasp_templates = &grasp_templates[uuid_database];
	for(unsigned int i = 0; i < tmp_grasp_templates->size(); ++i){
		if ((*tmp_grasp_templates)[i].uuid_database_template == uuid_database_template){
			return (*tmp_grasp_templates)[i];
		}
	}
	return Data_grasp();
}

void Database_grasp::Get_grasps(const size_t uuid_database,std::vector<Data_grasp> &result_database_grasps){
	_Get_grasps(uuid_database,result_database_grasps,_grasp_templates_success);
	if (result_database_grasps.size()>0){
		return;
	}
	_Get_grasps(uuid_database,result_database_grasps,_grasp_templates_fail);
}
void Database_grasp::_Get_grasps(const size_t uuid_database,std::vector<Data_grasp> &result_database_grasps,database_type &grasp_templates){
	std::vector<Data_grasp> tmp_grasp_templates = grasp_templates[uuid_database];
	result_database_grasps.resize(tmp_grasp_templates.size());
	std::copy(tmp_grasp_templates.begin(),tmp_grasp_templates.end(),result_database_grasps.begin());
}

void Database_grasp::Get_grasps(std::vector<Data_grasp> &result_database_grasps){
	_Get_grasps(result_database_grasps,_grasp_templates_success);
	_Get_grasps(result_database_grasps,_grasp_templates_fail);
}

void Database_grasp::_Get_grasps(std::vector<Data_grasp> &result_database_grasps,database_type &grasp_templates){
	database_type::iterator iter;
	for (iter = grasp_templates.begin(); iter != grasp_templates.end(); ++iter) {
		std::vector<Data_grasp> *tmp_vector = &iter->second;
		for (unsigned int i = 0; i < tmp_vector->size(); ++i) {
			result_database_grasps.push_back((*tmp_vector)[i]);
		}

	}
}

void Database_grasp::Get_grasp_templates(std::vector<Data_grasp> &result_database_grasps){
	database_type::iterator iter;
	for (iter = _grasp_templates_success.begin(); iter != _grasp_templates_success.end(); ++iter) {
		std::vector<Data_grasp> *tmp_vector = &iter->second;
		if (tmp_vector->size() > 0) {
			result_database_grasps.push_back((*tmp_vector)[0]);
		}

	}
}

}
