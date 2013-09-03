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

#ifndef UITLS_H
#define UTILS_H

#include <deep_learning/data_grasp.h>
#include <boost/functional/hash.hpp>
#include <geometry_msgs/Pose.h>
#include <grasp_template/Heightmap.h>
#include <iostream>

namespace deep_learning {

inline int transform_success(double success){
	if( success < 0.5){
		return SUCCESS_FALSE;
	}
	if( success > 0.5){
		return SUCCESS_TRUE;
	}
	return SUCCESS_FALSE;
};

	struct grasp_database_template_hash{
		const std::vector<double> *gripper_joints;
		const geometry_msgs::Pose *gripper_pose;
		const grasp_template::TemplateHeightmap *heightmap;
		grasp_database_template_hash(const std::vector<double> *pgripper_joints,const geometry_msgs::Pose *pgripper_pose,const grasp_template::TemplateHeightmap *pheightmap){
			gripper_joints = pgripper_joints;
			gripper_pose = pgripper_pose;
			heightmap = pheightmap;
		}
		grasp_database_template_hash(const Data_grasp &dgrasp){
			gripper_joints = &dgrasp.gripper_joints;
			gripper_pose = &dgrasp.gripper_pose;
			heightmap = &dgrasp.grasp_template.heightmap_;
		}
	};

	inline std::size_t hash_value(grasp_database_template_hash const& dgrasp){
		std::size_t seed = 0;
		for(unsigned int i = 0; i < dgrasp.gripper_joints->size();++i){
			boost::hash_combine(seed,(*dgrasp.gripper_joints)[i]);
		}

		for (unsigned int ix = 0; ix < dgrasp.heightmap->getNumTilesX(); ++ix) {
			for (unsigned int iy = 0; iy < dgrasp.heightmap->getNumTilesY(); ++iy) {
				boost::hash_combine(seed,dgrasp.heightmap->getGridTileRaw(ix,iy));
			}
		}
		boost::hash_combine(seed,dgrasp.gripper_pose->position.x);
		boost::hash_combine(seed,dgrasp.gripper_pose->position.y);
		boost::hash_combine(seed,dgrasp.gripper_pose->position.z);
		boost::hash_combine(seed,dgrasp.gripper_pose->orientation.x);
		boost::hash_combine(seed,dgrasp.gripper_pose->orientation.y);
		boost::hash_combine(seed,dgrasp.gripper_pose->orientation.z);
		boost::hash_combine(seed,dgrasp.gripper_pose->orientation.w);
		return seed;
	}

	struct grasp_database_hash{
		const std::vector<double> *gripper_joints;
		const geometry_msgs::Pose *gripper_pose;
		grasp_database_hash(const std::vector<double> *pgripper_joints,const geometry_msgs::Pose *pgripper_pose){
			gripper_joints = pgripper_joints;
			gripper_pose = pgripper_pose;
		}
		grasp_database_hash(const Data_grasp &dgrasp){
			gripper_joints = &dgrasp.gripper_joints;
			gripper_pose = &dgrasp.gripper_pose;
		}
	};

	inline std::size_t hash_value(grasp_database_hash const& dgrasp){
		std::size_t seed = 0;
		for(unsigned int i = 0; i < dgrasp.gripper_joints->size();++i){
			boost::hash_combine(seed,(int)((*dgrasp.gripper_joints)[i]*1000));
		}
		boost::hash_combine(seed,(int)(dgrasp.gripper_pose->position.x*1000));
		boost::hash_combine(seed,(int)(dgrasp.gripper_pose->position.y*1000));
		boost::hash_combine(seed,(int)(dgrasp.gripper_pose->position.z*1000));
		boost::hash_combine(seed,(int)(dgrasp.gripper_pose->orientation.x*1000));
		boost::hash_combine(seed,(int)(dgrasp.gripper_pose->orientation.y*1000));
		boost::hash_combine(seed,(int)(dgrasp.gripper_pose->orientation.z*1000));
		boost::hash_combine(seed,(int)(dgrasp.gripper_pose->orientation.w*1000));
		return seed;
	}

}

#endif /*DEF_H*/
