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

#ifndef DATA_GRASP_H
#define DATA_GRASP_H

#include <opencv2/core/core.hpp>
#include <grasp_template/grasp_template.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

class Data_grasp {
private:
public:
	geometry_msgs::Pose gripper_pose;
	grasp_template::GraspTemplate grasp_template;
	std::string uuid;
	float success;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Data_grasp(const geometry_msgs::Pose &pgripper_pose,const grasp_template::GraspTemplate &pgrasp_template,const std::string &puuid,float psuccess);
	Data_grasp(const geometry_msgs::Pose &pgripper_pose,const grasp_template::GraspTemplate &pgrasp_template);
	virtual ~Data_grasp() {
	}
	;
};

#endif /*DATA_STORAGE_H*/
