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

#ifndef DATABASE_GRASP_H
#define DATABASE_GRASP_H

#include <map>
#include <vector>
#include <deep_learning/data_grasp.h>

namespace deep_learning {
typedef std::map<std::size_t,std::vector<Data_grasp> > database_type;

class Database_grasp {
private:
	database_type _grasp_templates_success;
	database_type _grasp_templates_fail;
	void _Add_grasp(Data_grasp &grasp_template,database_type &grasp_templates);
	Data_grasp _Get_grasp(const size_t uuid_database,const size_t uuid_database_template, database_type &grasp_templates);
	void _Get_grasps(std::vector<Data_grasp> &result_database_grasps,database_type &database_type);
	void _Get_grasps(const size_t uuid_database,std::vector<Data_grasp> &result_database_grasps, database_type &database_type);
public:
	Database_grasp();
	virtual ~Database_grasp();

	void Add_grasp(Data_grasp &grasp_template);

	Data_grasp Get_grasp(const size_t uuid_database,const size_t uuid_database_template);
	void Get_grasps(const size_t uuid_database,std::vector<Data_grasp> &result_database_grasps);
	void Get_grasps(std::vector<Data_grasp> &result_database_grasps);
	void Get_grasps_success(std::vector<Data_grasp> &result_database_grasps);
	void Get_grasps_fail(std::vector<Data_grasp> &result_database_grasps);
	void Get_grasp_templates(std::vector<Data_grasp> &result_database_grasps);
};
}

#endif /*DATABASE_GRASP_H*/
